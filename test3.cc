#include "pinocchio/codegen/cppadcg.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include <iostream>
#include <fstream>

using namespace CppAD;
using namespace CppAD::cg;

int main(int argc, char **argv)
{
    // Use special types for source code generation
    using CGD = CG<double>;
    using ADCG = AD<CGD>;
    using namespace pinocchio;

    typedef ModelTpl<ADCG> ADModel;
    typedef DataTpl<ADCG> ADData;
    typedef Eigen::Matrix<ADCG, Eigen::Dynamic, 1> ADVectorXs;

    // Load or create a model
    Model model;
    if (argc > 1)
    {
        // Load from URDF file if provided
        std::string urdf_filename = argv[1];
        pinocchio::urdf::buildModel(urdf_filename, model);
        std::cout << "Loaded model from: " << urdf_filename << std::endl;
    }

    std::cout << "Model has " << model.njoints << " joints and " << model.nq << " DOF" << std::endl;

    // Convert to AD model for code generation
    ADModel ad_model = model.cast<ADCG>();
    ADData ad_data(ad_model);

    /***************************************************************************
     *                           Define the computation
     **************************************************************************/

    // Independent variables: joint configuration
    ADVectorXs ad_q;
    ad_q = ADVectorXs(ad_model.nq);
    Independent(ad_q);

    // Forward kinematics computation
    pinocchio::forwardKinematics(ad_model, ad_data, ad_q);

    // Collect all link poses into output vector
    // Each pose is represented as [x, y, z, qw, qx, qy, qz] (position + quaternion)
    const int pose_size = 3;
    ADVectorXs ad_poses = ADVectorXs(ad_model.njoints * pose_size);

    // Extract pose for each joint/link
    for (size_t i = 0; i < model.njoints; ++i)
    {
        const auto &pose = ad_data.oMi[i];

        // Store translation (x, y, z)
        ad_poses[i * pose_size + 0] = pose.translation()[0];
        ad_poses[i * pose_size + 1] = pose.translation()[1];
        ad_poses[i * pose_size + 2] = pose.translation()[2];
    }

    // Create the function tape
    ADFun<CGD> fun(ad_q, ad_poses);

    /***************************************************************************
     *                        Generate the C source code
     **************************************************************************/

    std::cout << "Generating C++ code..." << std::endl;

    CodeHandler<double> handler;
    CppAD::vector<CGD> indVars(ad_model.nq);
    handler.makeVariables(indVars);

    // Evaluate the function
    CppAD::vector<CGD> result = fun.Forward(0, indVars);

    LanguageC<double> langC("double");
    LangCDefaultVariableNameGenerator<double> nameGen;

    std::ostringstream code_stream;
    handler.generateCode(code_stream, langC, result, nameGen);

    /***************************************************************************
     *                         Create complete C++ file
     **************************************************************************/

    std::ostringstream complete_code;

    // Add includes and function header
    complete_code << "#include <iostream>\n";
    complete_code << "#include <vector>\n";
    complete_code << "#include <cmath>\n\n";

    // Add custom function declaration (user can modify this)
    complete_code << "// Custom function to call on each link pose\n";
    complete_code << "// pose: [x, y, z, qw, qx, qy, qz]\n";
    complete_code << "void processLinkPose(int link_id, const double pose[7]) {\n";
    complete_code << "    std::cout << \"Link \" << link_id << \" pose: \"\n";
    complete_code << "              << \"pos=[\" << pose[0] << \", \" << pose[1] << \", \" << pose[2] << \"] "
                     "\"\n";
    complete_code << "              << \"quat=[\" << pose[3] << \", \" << pose[4] << \", \" << pose[5] << "
                     "\", \" << pose[6] << \"]\\n\";\n";
    complete_code << "    // Add your custom processing here\n";
    complete_code << "}\n\n";

    // Add the generated forward kinematics function
    complete_code << "// Generated forward kinematics function\n";
    complete_code << "void computeForwardKinematics(const double* q, double* poses) {\n";

    // Insert the generated code
    std::string generated_code = code_stream.str();

    // Process the generated code to assign to output array
    complete_code << "    // Generated computation\n";
    complete_code << generated_code;
    complete_code << "\n";

    // Map the computed values to the output array
    complete_code << "    // Map results to output array\n";
    for (size_t i = 0; i < result.size(); ++i)
    {
        complete_code << "    poses[" << i << "] = v" << i << ";\n";
    }

    complete_code << "}\n\n";

    // Add main function wrapper
    complete_code << "// Wrapper function that calls custom function on each link\n";
    complete_code << "void forwardKinematicsWithCallback(const double* q) {\n";
    complete_code << "    const int num_joints = " << model.njoints << ";\n";
    complete_code << "    const int pose_size = 7;\n";
    complete_code << "    std::vector<double> poses(num_joints * pose_size);\n";
    complete_code << "\n";
    complete_code << "    // Compute all poses\n";
    complete_code << "    computeForwardKinematics(q, poses.data());\n";
    complete_code << "\n";
    complete_code << "    // Call custom function on each link pose\n";
    complete_code << "    for(int i = 0; i < num_joints; ++i) {\n";
    complete_code << "        processLinkPose(i, &poses[i * pose_size]);\n";
    complete_code << "    }\n";
    complete_code << "}\n\n";

    // Add example main function
    complete_code << "// Example usage\n";
    complete_code << "int main() {\n";
    complete_code << "    // Example joint configuration (modify as needed)\n";
    complete_code << "    std::vector<double> q(" << model.nq << ", 0.0);\n";
    complete_code << "    \n";
    complete_code << "    // Set some example joint values\n";
    complete_code << "    for(size_t i = 0; i < q.size(); ++i) {\n";
    complete_code << "        q[i] = 0.1 * i; // Example values\n";
    complete_code << "    }\n";
    complete_code << "    \n";
    complete_code << "    // Compute forward kinematics and call custom function on each link\n";
    complete_code << "    forwardKinematicsWithCallback(q.data());\n";
    complete_code << "    \n";
    complete_code << "    return 0;\n";
    complete_code << "}\n";

    /***************************************************************************
     *                             Output the code
     **************************************************************************/

    // Write to file
    std::string output_filename = "generated_forward_kinematics.cpp";
    std::ofstream outfile(output_filename);
    if (outfile.is_open())
    {
        outfile << complete_code.str();
        outfile.close();
        std::cout << "Generated code written to: " << output_filename << std::endl;
    }
    else
    {
        std::cerr << "Error: Could not write to file " << output_filename << std::endl;
        return 1;
    }

    // Also print to console for immediate viewing
    std::cout << "\n=== Generated C++ Code ===\n";
    std::cout << complete_code.str() << std::endl;

    // Print summary
    std::cout << "\n=== Summary ===\n";
    std::cout << "Model joints: " << model.njoints << std::endl;
    std::cout << "Model DOF: " << model.nq << std::endl;
    std::cout << "Generated variables: " << result.size() << std::endl;
    std::cout << "Output file: " << output_filename << std::endl;
    std::cout << "\nTo compile the generated code:\n";
    std::cout << "g++ -O3 -o fk_generated " << output_filename << std::endl;

    return 0;
}
