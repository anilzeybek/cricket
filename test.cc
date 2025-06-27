#include "pinocchio/codegen/cppadcg.hpp"  // Must be included first!

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/multibody/geometry.hpp"

#include <iostream>
#include <vector>
#include <fstream>

#include <coal/shape/geometric_shapes.h>

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

class CollisionSphereCodeGen
{
private:
    Model model_;
    GeometryModel collision_model_;
    std::vector<SphereInfo> sphere_infos_;

public:
    CollisionSphereCodeGen(const std::string &urdf_filename, const std::string &package_dir = "")
    {
        // Load model and collision geometry
        if (package_dir.empty())
        {
            pinocchio::urdf::buildModel(urdf_filename, model_);
            pinocchio::urdf::buildGeom(model_, urdf_filename, COLLISION, collision_model_);
        }
        else
        {
            std::vector<std::string> package_dirs = {package_dir};
            pinocchio::urdf::buildModel(urdf_filename, model_);
            pinocchio::urdf::buildGeom(model_, urdf_filename, COLLISION, collision_model_, package_dirs);
        }

        extractSphereInfo();
    }

    void extractSphereInfo()
    {
        std::cout << "Extracting sphere collision geometries..." << std::endl;

        for (GeomIndex i = 0; i < collision_model_.ngeoms; ++i)
        {
            const GeometryObject &geom_obj = collision_model_.geometryObjects[i];

            std::cout << "Checking geo..." << std::endl;
            // Check if the collision geometry is a sphere
            auto sphere_ptr = std::dynamic_pointer_cast<coal::Sphere>(geom_obj.geometry);
            if (sphere_ptr)
            {
                SphereInfo info;
                info.name = geom_obj.name;
                info.geom_index = i;
                info.radius = sphere_ptr->radius;
                info.parent_joint = geom_obj.parentJoint;
                info.local_placement = geom_obj.placement;

                sphere_infos_.push_back(info);
                std::cout << "Found sphere: " << info.name << " (radius: " << info.radius << ")" << std::endl;
            }
        }

        std::cout << "Total spheres found: " << sphere_infos_.size() << std::endl;
    }

    void generateCode(const std::string &output_filename = "collision_sphere_fk")
    {
        if (sphere_infos_.empty())
        {
            std::cout << "No sphere collision geometries found. Code generation skipped." << std::endl;
            return;
        }

        std::cout << "Generating collision sphere forward kinematics code..." << std::endl;

        // Convert to AD model
        ADModel ad_model = model_.cast<ADCG>();
        ADData ad_data(ad_model);

        // Set up independent variables (joint configuration)
        ADVectorXs ad_q(ad_model.nq);
        for (int i = 0; i < ad_model.nq; ++i)
        {
            ad_q[i] = ADCG(0.0);  // Initialize
        }

        // Mark as independent variables
        Independent(ad_q);

        // Perform forward kinematics
        forwardKinematics(ad_model, ad_data, ad_q);

        // Extract sphere positions and radii manually
        ADVectorXs sphere_data(sphere_infos_.size() * 4);  // 3 for position + 1 for radius

        for (size_t i = 0; i < sphere_infos_.size(); ++i)
        {
            const SphereInfo &info = sphere_infos_[i];

            // Get the joint placement in world frame
            const auto &joint_placement = ad_data.oMi[info.parent_joint];

            // Apply local geometry placement
            // Convert local placement to AD types
            Eigen::Matrix<ADCG, 3, 1> local_translation;
            local_translation[0] = ADCG(info.local_placement.translation()[0]);
            local_translation[1] = ADCG(info.local_placement.translation()[1]);
            local_translation[2] = ADCG(info.local_placement.translation()[2]);

            Eigen::Matrix<ADCG, 3, 3> local_rotation;
            for (int row = 0; row < 3; ++row)
            {
                for (int col = 0; col < 3; ++col)
                {
                    local_rotation(row, col) = ADCG(info.local_placement.rotation()(row, col));
                }
            }

            // Transform local geometry placement to world frame
            Eigen::Matrix<ADCG, 3, 1> world_position =
                joint_placement.rotation() * local_translation + joint_placement.translation();

            // Extract position (x, y, z)
            sphere_data[i * 4 + 0] = world_position[0];
            sphere_data[i * 4 + 1] = world_position[1];
            sphere_data[i * 4 + 2] = world_position[2];

            // Radius (constant)
            sphere_data[i * 4 + 3] = ADCG(info.radius);
        }

        // Create the AD function
        ADFun<CGD> collision_sphere_func(ad_q, sphere_data);

        // Generate C++ code
        generateCppCode(collision_sphere_func, output_filename);

        std::cout << "Code generation completed!" << std::endl;
    }

private:
    void generateCppCode(ADFun<CGD> &func, const std::string &base_filename)
    {
        // Create code handler
        CodeHandler<double> handler;

        // Create independent variables for code generation
        CppAD::vector<CGD> indVars(model_.nq);
        handler.makeVariables(indVars);

        // Evaluate function
        CppAD::vector<CGD> result = func.Forward(0, indVars);

        // Generate C++ code
        LanguageC<double> langC("double");
        LangCDefaultVariableNameGenerator<double> nameGen;

        // Generate function code
        std::ostringstream function_code;
        handler.generateCode(function_code, langC, result, nameGen);

        // Generate header file
        generateHeaderFile(base_filename + ".hpp");

        // Generate source file
        generateSourceFile(base_filename + ".cpp", function_code.str());

        std::cout << "Generated files: " << base_filename << ".hpp, " << base_filename << ".cpp" << std::endl;
    }

    void generateHeaderFile(const std::string &filename)
    {
        std::ofstream header_file(filename);
        if (!header_file.is_open())
        {
            throw std::runtime_error("Could not open header file for writing: " + filename);
        }

        header_file << "#ifndef COLLISION_SPHERE_FK_HPP\n";
        header_file << "#define COLLISION_SPHERE_FK_HPP\n\n";
        header_file << "#include <vector>\n";
        header_file << "#include <string>\n\n";
        header_file << "class CollisionSphereFKEvaluator {\n";
        header_file << "public:\n";
        header_file << "    // Number of spheres: " << sphere_infos_.size() << "\n";
        header_file << "    // Number of joints: " << model_.nq << "\n";
        header_file << "    static constexpr int NUM_SPHERES = " << sphere_infos_.size() << ";\n";
        header_file << "    static constexpr int NUM_JOINTS = " << model_.nq << ";\n";
        header_file << "    static constexpr int OUTPUT_SIZE = " << sphere_infos_.size() * 4 << ";\n\n";

        header_file << "    struct SphereData {\n";
        header_file << "        double x, y, z, radius;\n";
        header_file << "    };\n\n";

        header_file << "    // Generated function to compute sphere positions and radii\n";
        header_file << "    static void computeSphereFKGenerated(const double* q, double* sphere_data);\n\n";

        header_file << "    // Convenience function that returns structured data\n";
        header_file << "    static std::vector<SphereData> computeSphereFKStructured(const "
                       "std::vector<double>& q);\n\n";

        header_file << "    // Get sphere names\n";
        header_file << "    static std::vector<std::string> getSphereNames();\n";
        header_file << "};\n\n";
        header_file << "#endif // COLLISION_SPHERE_FK_HPP\n";

        header_file.close();
    }

    void generateSourceFile(const std::string &filename, const std::string &generated_code)
    {
        std::ofstream source_file(filename);
        if (!source_file.is_open())
        {
            throw std::runtime_error("Could not open source file for writing: " + filename);
        }

        std::string header_name = filename.substr(0, filename.find('.')) + ".hpp";
        source_file << "#include \"" << header_name << "\"\n";
        source_file << "#include <cmath>\n\n";

        // Insert the generated computation code
        source_file << "void CollisionSphereFKEvaluator::computeSphereFKGenerated(const double* q, double* "
                       "y) {\n";
        source_file << "    // Generated code for collision sphere forward kinematics\n";

        // Parse and clean up the generated code
        std::string cleaned_code = generated_code;
        // Replace any problematic variable names if needed
        // The generated code should use 'q' for input and 'y' for output

        source_file << "    " << cleaned_code << "\n";
        source_file << "}\n\n";

        // Add convenience function
        source_file << "std::vector<CollisionSphereFKEvaluator::SphereData> "
                       "CollisionSphereFKEvaluator::computeSphereFKStructured(const std::vector<double>& q) "
                       "{\n";
        source_file << "    double sphere_data[OUTPUT_SIZE];\n";
        source_file << "    computeSphereFKGenerated(q.data(), sphere_data);\n\n";
        source_file << "    std::vector<SphereData> result(NUM_SPHERES);\n";
        source_file << "    for (int i = 0; i < NUM_SPHERES; ++i) {\n";
        source_file << "        result[i].x = sphere_data[i * 4 + 0];\n";
        source_file << "        result[i].y = sphere_data[i * 4 + 1];\n";
        source_file << "        result[i].z = sphere_data[i * 4 + 2];\n";
        source_file << "        result[i].radius = sphere_data[i * 4 + 3];\n";
        source_file << "    }\n";
        source_file << "    return result;\n";
        source_file << "}\n\n";

        // Add sphere names function
        source_file << "std::vector<std::string> CollisionSphereFKEvaluator::getSphereNames() {\n";
        source_file << "    return {\n";
        for (size_t i = 0; i < sphere_infos_.size(); ++i)
        {
            source_file << "        \"" << sphere_infos_[i].name << "\"";
            if (i < sphere_infos_.size() - 1)
            {
                source_file << ",";
            }
            source_file << "\n";
        }
        source_file << "    };\n";
        source_file << "}\n";

        source_file.close();
    }

public:
    void printSphereInfo() const
    {
        std::cout << "\n=== Collision Sphere Information ===" << std::endl;
        for (const auto &info : sphere_infos_)
        {
            std::cout << "Sphere: " << info.name << std::endl;
            std::cout << "  Radius: " << info.radius << std::endl;
            std::cout << "  Parent Joint: " << model_.names[info.parent_joint] << std::endl;
            std::cout << "  Local Placement: " << info.local_placement.translation().transpose() << std::endl;
            std::cout << std::endl;
        }
    }
};

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " <urdf_file> [package_directory] [output_name]" << std::endl;
        return 1;
    }

    std::string urdf_file = argv[1];
    std::string package_dir = (argc > 2) ? argv[2] : "";
    std::string output_name = (argc > 3) ? argv[3] : "collision_sphere_fk";

    try
    {
        // Create code generator
        CollisionSphereCodeGen codegen(urdf_file, package_dir);

        // Print information about found spheres
        codegen.printSphereInfo();

        // Generate optimized C++ code
        codegen.generateCode(output_name);

        std::cout << "\nCode generation successful!" << std::endl;
        std::cout << "Generated files can be compiled and used in your projects." << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
