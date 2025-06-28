#include "pinocchio/codegen/cppadcg.hpp"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/multibody/geometry.hpp"

#include <filesystem>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <vector>

#include <fmt/core.h>
#include <nlohmann/json.hpp>
#include <inja/inja.hpp>

#include <coal/shape/geometric_shapes.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Min_sphere_of_spheres_d.h>
#include <CGAL/Min_sphere_of_spheres_d_traits_3.h>

using namespace pinocchio;
using namespace CppAD;
using namespace CppAD::cg;

// Typedef for AD types
using CGD = CG<double>;
using ADCG = AD<CGD>;
using ADModel = ModelTpl<ADCG>;
using ADData = DataTpl<ADCG>;
using ADVectorXs = Eigen::Matrix<ADCG, Eigen::Dynamic, 1>;

struct SphereInfo
{
    std::string name;
    GeomIndex geom_index;
    double radius;
    JointIndex parent_joint;
    SE3 relative;
};

auto min_sphere_of_spheres(const std::vector<SphereInfo> &info) -> std::array<float, 4>
{
    using K = CGAL::Exact_predicates_inexact_constructions_kernel;
    using Traits = CGAL::Min_sphere_of_spheres_d_traits_3<K, double>;
    using Sphere = Traits::Sphere;
    using Point = K::Point_3;
    using MinSphere = CGAL::Min_sphere_of_spheres_d<Traits>;

    std::vector<Sphere> cgal_spheres;
    cgal_spheres.reserve(info.size());

    for (const auto &sphere : info)
    {
        auto pos = sphere.relative.translation();
        cgal_spheres.emplace_back(Point(pos[0], pos[1], pos[2]), sphere.radius);
    }

    MinSphere ms(cgal_spheres.begin(), cgal_spheres.end());
    std::array<float, 4> sphere;
    std::copy(ms.center_cartesian_begin(), ms.center_cartesian_end(), sphere.begin());
    sphere[3] = ms.radius();
    return sphere;
}

struct RobotInfo
{
    RobotInfo(
        const std::filesystem::path &urdf_file,
        const std::filesystem::path &srdf_file,
        const std::string &end_effector)
    {
        pinocchio::urdf::buildModel(urdf_file, model);
        pinocchio::urdf::buildGeom(model, urdf_file, COLLISION, collision_model);

        collision_model.addAllCollisionPairs();
        pinocchio::srdf::removeCollisionPairs(model, collision_model, srdf_file);

        extract_spheres();
        extract_collision_data();

        end_effector_name = end_effector;
        const auto ee_index_itr = link_name_to_index.find(end_effector);
        if (ee_index_itr == link_name_to_index.end())
        {
            throw std::runtime_error(fmt::format("Invalid EE name {}", end_effector));
        }

        end_effector_index = ee_index_itr->second;

        nq = model.nq;

        lower_bound = model.lowerPositionLimit;
        upper_bound = model.upperPositionLimit;
    }

    auto json() -> nlohmann::json
    {
        Eigen::VectorXd bound_range = upper_bound - lower_bound;
        Eigen::VectorXd bound_descale = bound_range.cwiseInverse();

        nlohmann::json json;
        json["name"] = model.name;
        json["n_q"] = nq;
        json["n_spheres"] = nq;
        json["bound_lower"] = std::vector<float>(lower_bound.data(), lower_bound.data() + nq);
        json["bound_range"] = std::vector<float>(bound_range.data(), bound_range.data() + nq);
        json["bound_descale"] = std::vector<float>(bound_descale.data(), bound_descale.data() + nq);
        json["measure"] = bound_range.prod();

        return json;
    }

    auto extract_spheres() -> void
    {
        for (auto i = 0U; i < collision_model.ngeoms; ++i)
        {
            const auto &geom_obj = collision_model.geometryObjects[i];
            auto sphere_ptr = std::dynamic_pointer_cast<coal::Sphere>(geom_obj.geometry);
            if (sphere_ptr)
            {
                SphereInfo info;
                info.name = geom_obj.name;
                info.geom_index = i;
                info.radius = sphere_ptr->radius;
                info.parent_joint = geom_obj.parentJoint;
                info.relative = geom_obj.placement;

                spheres.emplace_back(info);
            }
            else
            {
                throw std::runtime_error(
                    fmt::format("Invalid non-sphere geometry in URDF {}", geom_obj.name));
            }
        }

        for (auto i = 0U; i < model.joints.size(); ++i)
        {
            std::vector<SphereInfo> link_info;
            for (const auto &info : spheres)
            {
                if (info.parent_joint == i)
                {
                    link_info.emplace_back(info);
                }
            }

            per_link_spheres.emplace_back(link_info);

            if (not link_info.empty())
            {
                auto sphere = min_sphere_of_spheres(link_info);

                SphereInfo info;
                info.name = fmt::format("{}_bounding_sphere", model.names[i]);
                info.geom_index = collision_model.ngeoms + i;
                info.radius = sphere[3];
                info.parent_joint = i;
                info.relative = SE3::Identity();
                info.relative.translation()[0] = sphere[0];
                info.relative.translation()[1] = sphere[1];
                info.relative.translation()[2] = sphere[2];

                bounding_spheres.emplace_back(info);
            }
        }
    }

    auto extract_collision_data() -> void
    {
        for (FrameIndex i = 0; i < model.frames.size(); ++i)
        {
            if (model.frames[i].type == BODY)
            {
                link_name_to_index[model.frames[i].name] = i;
            }
        }

        for (size_t k = 0; k < collision_model.collisionPairs.size(); ++k)
        {
            const CollisionPair &cp = collision_model.collisionPairs[k];

            const GeometryObject &geom1 = collision_model.geometryObjects[cp.first];
            const GeometryObject &geom2 = collision_model.geometryObjects[cp.second];

            const std::string &link1_name = model.frames[geom1.parentFrame].name;
            const std::string &link2_name = model.frames[geom2.parentFrame].name;

            FrameIndex link1_idx = geom1.parentFrame;
            FrameIndex link2_idx = geom2.parentFrame;

            FrameIndex first_idx = std::min(link1_idx, link2_idx);
            FrameIndex second_idx = std::max(link1_idx, link2_idx);
            allowed_link_pairs.insert(std::make_pair(first_idx, second_idx));

            link_collision_map[link1_name].insert(link2_name);
            link_collision_map[link2_name].insert(link1_name);
        }
    }

    Model model;
    GeometryModel collision_model;
    std::string end_effector_name;
    std::size_t end_effector_index;
    std::size_t nq;

    Eigen::VectorXd upper_bound;
    Eigen::VectorXd lower_bound;

    std::vector<SphereInfo> spheres;
    std::vector<std::vector<SphereInfo>> per_link_spheres;
    std::vector<SphereInfo> bounding_spheres;

    std::map<std::string, FrameIndex> link_name_to_index;
    std::set<std::pair<FrameIndex, FrameIndex>> allowed_link_pairs;
    std::map<std::string, std::set<std::string>> link_collision_map;
};

auto trace_sphere(const SphereInfo &sphere, const ADData &ad_data, ADVectorXs &data, std::size_t index)
{
    const auto &joint_placement = ad_data.oMi[sphere.parent_joint];

    Eigen::Matrix<ADCG, 3, 1> local_translation;
    local_translation[0] = sphere.relative.translation()[0];
    local_translation[1] = sphere.relative.translation()[1];
    local_translation[2] = sphere.relative.translation()[2];

    Eigen::Matrix<ADCG, 3, 1> world_position =
        joint_placement.rotation() * local_translation + joint_placement.translation();

    data[index + 0] = world_position[0];
    data[index + 1] = world_position[1];
    data[index + 2] = world_position[2];
    data[index + 3] = ADCG(sphere.radius);
}

auto trace_frame(std::size_t ee_index, const ADData &ad_data, ADVectorXs &data, std::size_t index)
{
    const auto &oMf = ad_data.oMf[ee_index];
    const auto &R = oMf.rotation();

    // quaternion conversion from rotation matrix using the trace-based method which avoids conditionals
    ADCG trace = R(0, 0) + R(1, 1) + R(2, 2);
    ADCG s = sqrt(trace + 1.0 + 1e-10);  // Add small epsilon for numerical stability
    ADCG w = s * 0.5;
    ADCG inv_s = 1.0 / (2.0 * w);

    data[index + 0] = oMf.translation()[0];
    data[index + 1] = oMf.translation()[1];
    data[index + 2] = oMf.translation()[2];

    data[index + 3] = (R(2, 1) - R(1, 2)) * inv_s;  // x
    data[index + 4] = (R(0, 2) - R(2, 0)) * inv_s;  // y
    data[index + 5] = (R(1, 0) - R(0, 1)) * inv_s;  // z
    data[index + 6] = w;                            // w
}

struct Traced
{
    std::string code;
    std::size_t temp_variables;
    std::size_t outputs;
};

auto trace_fk(const RobotInfo &info) -> Traced
{
    auto nq = info.model.nq;
    ADModel ad_model = info.model.cast<ADCG>();
    ADData ad_data(ad_model);

    ADVectorXs ad_q(nq);
    for (auto i = 0U; i < nq; ++i)
    {
        ad_q[i] = ADCG(0.0);
    }

    Independent(ad_q);

    forwardKinematics(ad_model, ad_data, ad_q);
    updateFramePlacements(ad_model, ad_data);

    ADVectorXs data(7);

    trace_frame(
        info.end_effector_index, ad_data, data, 0);

    // Create the AD function
    ADFun<CGD> func(ad_q, data);

    CodeHandler<double> handler;
    CppAD::vector<CGD> ind_vars(nq);
    handler.makeVariables(ind_vars);

    CppAD::vector<CGD> result = func.Forward(0, ind_vars);

    LanguageC<double> langC("double");
    LangCDefaultVariableNameGenerator<double> nameGen;

    std::ostringstream function_code;
    handler.generateCode(function_code, langC, result, nameGen);

    return Traced{
        function_code.str(), nameGen.getMaxTemporaryVariableID(), 7};
}

auto trace_sphere_fk(const RobotInfo &info) -> std::string
{
    auto nq = info.model.nq;
    ADModel ad_model = info.model.cast<ADCG>();
    ADData ad_data(ad_model);

    ADVectorXs ad_q(nq);
    for (auto i = 0U; i < nq; ++i)
    {
        ad_q[i] = ADCG(0.0);
    }

    Independent(ad_q);

    forwardKinematics(ad_model, ad_data, ad_q);
    updateFramePlacements(ad_model, ad_data);

    ADVectorXs data((info.bounding_spheres.size() + info.spheres.size()) * 4 + 7);

    for (auto i = 0U; i < info.spheres.size(); ++i)
    {
        trace_sphere(info.spheres[i], ad_data, data, i * 4);
    }

    for (auto i = 0U; i < info.bounding_spheres.size(); ++i)
    {
        trace_sphere(info.bounding_spheres[i], ad_data, data, (i + info.spheres.size()) * 4);
    }

    trace_frame(
        info.end_effector_index, ad_data, data, (info.bounding_spheres.size() + info.spheres.size()) * 4);

    // Create the AD function
    ADFun<CGD> collision_sphere_func(ad_q, data);

    CodeHandler<double> handler;
    CppAD::vector<CGD> ind_vars(nq);
    handler.makeVariables(ind_vars);

    CppAD::vector<CGD> result = collision_sphere_func.Forward(0, ind_vars);

    LanguageC<double> langC("double");
    LangCDefaultVariableNameGenerator<double> nameGen;

    std::ostringstream function_code;
    handler.generateCode(function_code, langC, result, nameGen);

    return function_code.str();
}

int main(int argc, char **argv)
{
    std::filesystem::path urdf_file = argv[1];
    std::filesystem::path srdf_file = argv[2];

    RobotInfo robot(urdf_file, srdf_file, argv[3]);

    auto json = robot.json();
    auto traced_eefk_code = trace_fk(robot);

    json["eefk_code"] = traced_eefk_code.code;
    json["eefk_code_vars"] = traced_eefk_code.temp_variables;

    inja::Environment env;
    inja::Template temp = env.parse_template("./fk_template.hh");
    env.write(temp, json, fmt::format("{}_fk.hh", robot.model.name));

    return 0;
}
