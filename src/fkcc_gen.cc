#include "pinocchio_cppadcg.hh"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <coal/shape/geometric_shapes.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Min_sphere_of_spheres_d.h>
#include <CGAL/Min_sphere_of_spheres_d_traits_3.h>

#include <fmt/core.h>
#include <nlohmann/json.hpp>
#include <inja/inja.hpp>

#include <filesystem>
#include <stdexcept>
#include <vector>

#include "lang_gen.hh"

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
    float radius;
    JointIndex parent_joint;
    JointIndex parent_frame;
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
        if (not std::filesystem::exists(urdf_file))
        {
            throw std::runtime_error(fmt::format("URDF file {} does not exist!", urdf_file.string()));
        }

        if (not std::filesystem::exists(srdf_file))
        {
            throw std::runtime_error(fmt::format("SRDF file {} does not exist!", srdf_file.string()));
        }

        pinocchio::urdf::buildModel(urdf_file, model);
        pinocchio::urdf::buildGeom(model, urdf_file, COLLISION, collision_model);

        collision_model.addAllCollisionPairs();
        pinocchio::srdf::removeCollisionPairs(model, collision_model, srdf_file);

        extract_spheres();
        extract_collision_data();

        end_effector_name = end_effector;

        if (not model.existFrame(end_effector))
        {
            throw std::runtime_error(fmt::format("Invalid EE name {}", end_effector));
        }

        end_effector_index = model.getFrameId(end_effector);
    }

    auto json() -> nlohmann::json
    {
        const Eigen::VectorXd lower_bound = model.lowerPositionLimit;
        const Eigen::VectorXd upper_bound = model.upperPositionLimit;
        const Eigen::VectorXd bound_range = upper_bound - lower_bound;
        const Eigen::VectorXd bound_descale = bound_range.cwiseInverse();

        nlohmann::json json;
        json["n_q"] = model.nq;
        json["n_spheres"] = spheres.size();
        json["bound_lower"] = std::vector<float>(lower_bound.data(), lower_bound.data() + model.nq);
        json["bound_range"] = std::vector<float>(bound_range.data(), bound_range.data() + model.nq);
        json["bound_descale"] = std::vector<float>(bound_descale.data(), bound_descale.data() + model.nq);
        json["measure"] = bound_range.prod();
        json["end_effector_index"] = end_effector_index;
        json["min_radius"] = min_radius;
        json["max_radius"] = max_radius;
        json["joint_names"] = dof_to_joint_names();
        json["allowed_link_pairs"] = allowed_link_pairs;
        json["per_link_spheres"] = per_link_spheres;
        json["links_with_geometry"] = links_with_geometry;
        json["bounding_sphere_index"] = bounding_sphere_index;
        json["end_effector_collisions"] = get_frames_colliding_end_effector();

        std::vector<std::string> link_names;
        for (auto i = 0U; i < model.frames.size(); ++i)
        {
            link_names.emplace_back(model.frames[i].name);
        }
        json["link_names"] = link_names;

        return json;
    }

    auto dof_to_joint_names() -> std::vector<std::string>
    {
        std::vector<std::size_t> dof_to_joint_id(model.nq);
        for (auto joint_id = 1U; joint_id < model.joints.size(); ++joint_id)
        {
            const auto &joint = model.joints[joint_id];
            auto start_idx = joint.idx_q();
            auto nq = joint.nq();

            for (auto i = 0U; i < nq; ++i)
            {
                dof_to_joint_id[start_idx + i] = joint_id;
            }
        }

        std::vector<std::string> dof_to_joint_name(model.nq);
        for (auto i = 0U; i < model.nq; ++i)
        {
            dof_to_joint_name[i] = model.names[dof_to_joint_id[i]];
        }

        return dof_to_joint_name;
    }

    auto get_frames_colliding_end_effector() -> std::vector<std::size_t>
    {
        std::size_t end_effector_joint = model.frames[end_effector_index].parentJoint;

        std::vector<std::size_t> frames;
        for (auto i = 0U; i < model.frames.size(); ++i)
        {
            if (model.frames[i].parentJoint == end_effector_joint)
            {
                if (bounding_spheres.find(i) != bounding_spheres.end())
                {
                    frames.emplace_back(i);
                }
            }
        }

        std::set<std::size_t> end_effector_allowed_collisions;
        for (const auto &[first, second] : allowed_link_pairs)
        {
            if (std::find(frames.begin(), frames.end(), first) != frames.end())
            {
                end_effector_allowed_collisions.emplace(second);
            }

            if (std::find(frames.begin(), frames.end(), second) != frames.end())
            {
                end_effector_allowed_collisions.emplace(first);
            }
        }

        return std::vector<std::size_t>(
            end_effector_allowed_collisions.begin(), end_effector_allowed_collisions.end());
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
                info.parent_frame = geom_obj.parentFrame;
                info.relative = geom_obj.placement;

                spheres.emplace_back(info);

                min_radius = std::min(min_radius, info.radius);
                max_radius = std::max(max_radius, info.radius);
            }
            else
            {
                throw std::runtime_error(
                    fmt::format("Invalid non-sphere geometry in URDF {}", geom_obj.name));
            }
        }

        std::size_t bs = 0;
        for (auto i = 0U; i < model.frames.size(); ++i)
        {
            std::vector<SphereInfo> link_info;
            std::vector<std::size_t> sphere_indices;
            for (const auto &info : spheres)
            {
                if (info.parent_frame == i)
                {
                    link_info.emplace_back(info);
                    sphere_indices.emplace_back(info.geom_index);
                }
            }

            per_link_spheres.emplace_back(sphere_indices);

            if (not link_info.empty())
            {
                auto sphere = min_sphere_of_spheres(link_info);

                SphereInfo info;
                info.name = fmt::format("{}_bounding_sphere", model.frames[i].name);
                info.geom_index = bs;
                info.radius = sphere[3];
                info.parent_joint = link_info[0].parent_joint;
                info.relative = SE3::Identity();
                info.relative.translation()[0] = sphere[0];
                info.relative.translation()[1] = sphere[1];
                info.relative.translation()[2] = sphere[2];

                bounding_spheres.emplace(i, info);
                bounding_sphere_index.emplace_back(bs);
                links_with_geometry.emplace_back(i);
                bs++;
            }
            else
            {
                bounding_sphere_index.emplace_back(0);
            }
        }
    }

    auto extract_collision_data() -> void
    {
        for (const auto &cp : collision_model.collisionPairs)
        {
            const auto &geom1 = collision_model.geometryObjects[cp.first];
            const auto &geom2 = collision_model.geometryObjects[cp.second];

            std::size_t link1_idx = geom1.parentFrame;
            std::size_t link2_idx = geom2.parentFrame;

            FrameIndex first_idx = std::min(link1_idx, link2_idx);
            FrameIndex second_idx = std::max(link1_idx, link2_idx);
            allowed_link_pairs.insert(std::make_pair(first_idx, second_idx));
        }
    }

    Model model;
    GeometryModel collision_model;
    std::string end_effector_name;
    std::size_t end_effector_index;

    float min_radius{std::numeric_limits<float>::max()};
    float max_radius{std::numeric_limits<float>::min()};
    std::vector<SphereInfo> spheres;
    std::map<std::size_t, SphereInfo> bounding_spheres;
    std::vector<std::size_t> links_with_geometry;
    std::vector<std::vector<std::size_t>> per_link_spheres;
    std::set<std::pair<std::size_t, std::size_t>> allowed_link_pairs;
    std::vector<std::size_t> bounding_sphere_index;
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

auto trace_sphere_cc_fk(
    const RobotInfo &info,
    bool spheres = true,
    bool bounding_spheres = true,
    bool fk = true) -> Traced
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

    std::size_t n_spheres_data = (spheres) ? info.spheres.size() * 4 : 0;
    std::size_t n_bounding_spheres_data = (bounding_spheres) ? info.bounding_spheres.size() * 4 : 0;
    std::size_t n_fk_data = (fk) ? 7 : 0;

    std::size_t n_out = n_spheres_data + n_bounding_spheres_data + n_fk_data;

    ADVectorXs data(n_out);

    if (spheres)
    {
        for (auto i = 0U; i < info.spheres.size(); ++i)
        {
            const auto &sphere = info.spheres[i];
            trace_sphere(sphere, ad_data, data, sphere.geom_index * 4);
        }
    }

    if (bounding_spheres)
    {
        for (auto i = 0U; i < info.model.frames.size(); ++i)
        {
            auto sphere_it = info.bounding_spheres.find(i);
            if (sphere_it != info.bounding_spheres.end())
            {
                const auto &sphere = sphere_it->second;
                trace_sphere(sphere, ad_data, data, sphere.geom_index * 4 + n_spheres_data);
            }
        }
    }

    if (fk)
    {
        trace_frame(info.end_effector_index, ad_data, data, n_spheres_data + n_bounding_spheres_data);
    }

    // Create the AD function
    ADFun<CGD> collision_sphere_func(ad_q, data);

    CodeHandler<double> handler;
    CppAD::vector<CGD> ind_vars(nq);
    handler.makeVariables(ind_vars);

    CppAD::vector<CGD> result = collision_sphere_func.Forward(0, ind_vars);

    LanguageCCustom<double> langC("double");
    LangCDefaultVariableNameGenerator<double> nameGen;

    std::ostringstream function_code;
    handler.generateCode(function_code, langC, result, nameGen);

    return Traced{function_code.str(), handler.getTemporaryVariableCount(), n_out};
}

int main(int argc, char **argv)
{
    std::filesystem::path json_path(argv[1]);
    auto parent_path = json_path.parent_path();

    if (not std::filesystem::exists(json_path))
    {
        throw std::runtime_error(fmt::format("JSON file {} does not exist!", json_path.string()));
    }

    std::ifstream f(json_path);
    nlohmann::json data = nlohmann::json::parse(f);

    RobotInfo robot(parent_path / data["urdf"], parent_path / data["srdf"], data["end_effector"]);

    data.update(robot.json());

    auto traced_eefk_code = trace_sphere_cc_fk(robot, false, false, true);
    data["eefk_code"] = traced_eefk_code.code;
    data["eefk_code_vars"] = traced_eefk_code.temp_variables;

    auto traced_spherefk_code = trace_sphere_cc_fk(robot, true, false, false);
    data["spherefk_code"] = traced_spherefk_code.code;
    data["spherefk_code_vars"] = traced_spherefk_code.temp_variables;
    data["spherefk_code_output"] = traced_spherefk_code.outputs;

    auto traced_ccfk_code = trace_sphere_cc_fk(robot, true, true, false);
    data["ccfk_code"] = traced_ccfk_code.code;
    data["ccfk_code_vars"] = traced_ccfk_code.temp_variables;
    data["ccfk_code_output"] = traced_ccfk_code.outputs;

    auto traced_ccfkee_code = trace_sphere_cc_fk(robot, true, true, true);
    data["ccfkee_code"] = traced_ccfkee_code.code;
    data["ccfkee_code_vars"] = traced_ccfkee_code.temp_variables;
    data["ccfkee_code_output"] = traced_ccfkee_code.outputs;

    inja::Environment env;

    for (const auto &subt : data["subtemplates"])
    {
        inja::Template temp = env.parse_template(parent_path / subt["template"]);
        env.include_template(subt["name"], temp);
    }

    inja::Template temp = env.parse_template(parent_path / data["template"]);
    env.write(temp, data, data["output"]);

    std::ofstream output_file("output.json");
    output_file << data.dump(4);  // 4 spaces indentation
    output_file.close();

    return 0;
}
