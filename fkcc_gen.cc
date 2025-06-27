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
#include <vector>
#include <fstream>

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
    SE3 local_placement;
};

auto min_sphere_of_spheres(const std::vector<SphereInfo> &info) -> SphereInfo
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
        cgal_spheres.emplace_back(
            Point(
                sphere.local_placement.translation()[0],
                sphere.local_placement.translation()[1],
                sphere.local_placement.translation()[2]),
            sphere.radius);
    }

    MinSphere ms(cgal_spheres.begin(), cgal_spheres.end());

    MinSphere::Cartesian_const_iterator ccib = ms.center_cartesian_begin(), ccie = ms.center_cartesian_end();
    std::cout << "center:";
    for (; ccib != ccie; ++ccib)
    {
        std::cout << " " << *ccib;
    }
    std::cout << std::endl << "radius: " << ms.radius() << std::endl;
    return SphereInfo{};
}

struct RobotInfo
{
    RobotInfo(const std::filesystem::path &urdf_file, const std::filesystem::path &srdf_file)
    {
        pinocchio::urdf::buildModel(urdf_file, model);
        pinocchio::urdf::buildGeom(model, urdf_file, COLLISION, collision_model);

        collision_model.addAllCollisionPairs();
        pinocchio::srdf::removeCollisionPairs(model, collision_model, srdf_file);
    }

    auto extract_spheres() -> std::vector<SphereInfo>
    {
        std::vector<SphereInfo> sphere_info;
        for (GeomIndex i = 0; i < collision_model.ngeoms; ++i)
        {
            const GeometryObject &geom_obj = collision_model.geometryObjects[i];

            auto sphere_ptr = std::dynamic_pointer_cast<coal::Sphere>(geom_obj.geometry);
            if (sphere_ptr)
            {
                SphereInfo info;
                info.name = geom_obj.name;
                info.geom_index = i;
                info.radius = sphere_ptr->radius;
                info.parent_joint = geom_obj.parentJoint;
                info.local_placement = geom_obj.placement;

                sphere_info.emplace_back(info);
            }
            else
            {
                std::cout << "issue! non sphere geo" << std::endl;
            }
        }

        for (JointIndex i = 0; i < model.joints.size(); ++i)
        {
            std::vector<SphereInfo> link_info;
            for (const auto &info : sphere_info)
            {
                if (info.parent_joint == i)
                {
                    link_info.emplace_back(info);
                }
            }

            if (not link_info.empty())
            {
                min_sphere_of_spheres(link_info);
            }
        }

        return sphere_info;
    }

    Model model;
    GeometryModel collision_model;
    std::vector<SphereInfo> bounding_sphere_info;
    std::vector<SphereInfo> sphere_info;
};

int main(int argc, char **argv)
{
    std::filesystem::path urdf_file = argv[1];
    std::filesystem::path srdf_file = argv[2];

    RobotInfo robot(urdf_file, srdf_file);

    std::vector<SphereInfo> sphere_info = robot.extract_spheres();
    return 0;
}

// sphere_info.insert(sphere_info.end(), sts.begin(), sts.end());
// sphere_info.insert(sphere_info.end(), stf.begin(), stf.end());

//     std::cout << "Total spheres found: " << sphere_info.size() << std::endl;

//     // std::map<std::string, FrameIndex> link_name_to_index;
//     // for (FrameIndex i = 0; i < model.frames.size(); ++i)
//     // {
//     //     if (model.frames[i].type == BODY)
//     //     {
//     //         link_name_to_index[model.frames[i].name] = i;
//     //     }
//     // }

//     std::set<std::pair<FrameIndex, FrameIndex>> allowed_link_pairs;
//     std::map<std::string, std::set<std::string>> link_collision_map;

//     for (size_t k = 0; k < collision_model.collisionPairs.size(); ++k)
//     {
//         const CollisionPair &cp = collision_model.collisionPairs[k];

//         const GeometryObject &geom1 = collision_model.geometryObjects[cp.first];
//         const GeometryObject &geom2 = collision_model.geometryObjects[cp.second];

//         const std::string &link1_name = model.frames[geom1.parentFrame].name;
//         const std::string &link2_name = model.frames[geom2.parentFrame].name;

//         FrameIndex link1_idx = geom1.parentFrame;
//         FrameIndex link2_idx = geom2.parentFrame;

//         FrameIndex first_idx = std::min(link1_idx, link2_idx);
//         FrameIndex second_idx = std::max(link1_idx, link2_idx);
//         allowed_link_pairs.insert(std::make_pair(first_idx, second_idx));

//         link_collision_map[link1_name].insert(link2_name);
//         link_collision_map[link2_name].insert(link1_name);
//     }

//     int pair_count = 0;
//     for (const auto &link_pair : allowed_link_pairs)
//     {
//         const std::string &link1_name = model.frames[link_pair.first].name;
//         const std::string &link2_name = model.frames[link_pair.second].name;

//         std::cout << "Pair " << pair_count++ << ": ";
//         std::cout << "(" << link_pair.first << ", " << link_pair.second << ") ";
//         std::cout << "-> '" << link1_name << "' <-> '" << link2_name << "'" << std::endl;
//     }

//     std::cout << "Tracing collision sphere forward kinematics code..." << std::endl;

//     ADModel ad_model = model.cast<ADCG>();
//     ADData ad_data(ad_model);

//     FrameIndex end_effector_frame = ad_model.frames.size() - 1;

//     ADVectorXs ad_q(ad_model.nq);
//     for (int i = 0; i < ad_model.nq; ++i)
//     {
//         ad_q[i] = ADCG(0.0);
//     }

//     Independent(ad_q);

//     forwardKinematics(ad_model, ad_data, ad_q);
//     updateFramePlacements(ad_model, ad_data);

//     ADVectorXs data(sphere_info.size() * 4 + 7);  // 3 for position + 1 for radius + EE pose

//     for (size_t i = 0; i < sphere_info.size(); ++i)
//     {
//         const SphereInfo &info = sphere_info[i];
//         const auto &joint_placement = ad_data.oMi[info.parent_joint];

//         Eigen::Matrix<ADCG, 3, 1> local_translation;
//         local_translation[0] = info.local_placement.translation()[0];
//         local_translation[1] = info.local_placement.translation()[1];
//         local_translation[2] = info.local_placement.translation()[2];

//         Eigen::Matrix<ADCG, 3, 1> world_position =
//             joint_placement.rotation() * local_translation + joint_placement.translation();

//         data[i * 4 + 0] = world_position[0];
//         data[i * 4 + 1] = world_position[1];
//         data[i * 4 + 2] = world_position[2];
//         data[i * 4 + 3] = ADCG(info.radius);
//     }

//     const auto &oMf = ad_data.oMf[end_effector_frame];

//     const auto &R = oMf.rotation();

//     // quaternion conversion from rotation matrix using the trace-based method which avoids conditionals
//     ADCG trace = R(0, 0) + R(1, 1) + R(2, 2);
//     ADCG s = sqrt(trace + 1.0 + 1e-10);  // Add small epsilon for numerical stability
//     ADCG w = s * 0.5;
//     ADCG inv_s = 1.0 / (2.0 * w);

//     data[sphere_info.size() * 4 + 0] = oMf.translation()[0];
//     data[sphere_info.size() * 4 + 1] = oMf.translation()[1];
//     data[sphere_info.size() * 4 + 2] = oMf.translation()[2];

//     data[sphere_info.size() * 4 + 3] = (R(2, 1) - R(1, 2)) * inv_s;  // x
//     data[sphere_info.size() * 4 + 4] = (R(0, 2) - R(2, 0)) * inv_s;  // y
//     data[sphere_info.size() * 4 + 5] = (R(1, 0) - R(0, 1)) * inv_s;  // z
//     data[sphere_info.size() * 4 + 6] = w;                            // w

//     // Create the AD function
//     ADFun<CGD> collision_sphere_func(ad_q, data);

//     std::cout << "Generating collision sphere fk code..." << std::endl;

//     CodeHandler<double> handler;
//     CppAD::vector<CGD> indVars(model.nq);
//     handler.makeVariables(indVars);

//     CppAD::vector<CGD> result = collision_sphere_func.Forward(0, indVars);

//     LanguageC<double> langC("double");
//     LangCDefaultVariableNameGenerator<double> nameGen;

//     std::ostringstream function_code;
//     handler.generateCode(function_code, langC, result, nameGen);

//     std::cout << "Code generation completed!" << std::endl;
//     std::cout << function_code.str() << std::endl;
// }
