#include "pinocchio/codegen/cppadcg.hpp"  // Must be included first!
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include <iostream>

using namespace pinocchio;
using namespace CppAD;
using namespace CppAD::cg;

int main(int argc, char **argv)
{
    // Load the URDF model
    std::string urdf_filename = (argc > 1) ? argv[1] : "panda/panda_spherized.urdf";

    Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);

    std::cout << "Model loaded with " << model.nq << " configuration variables" << std::endl;

    // Set up automatic differentiation types
    typedef CG<double> CGD;
    typedef AD<CGD> ADCG;
    typedef ModelTpl<ADCG> ADModel;
    typedef DataTpl<ADCG> ADData;
    typedef Eigen::Matrix<ADCG, Eigen::Dynamic, 1> ADVectorXs;
    typedef Eigen::Matrix<ADCG, 4, 4> ADMatrix4;

    // Convert model to automatic differentiation
    ADModel ad_model = model.cast<ADCG>();
    ADData ad_data(ad_model);

    // Define independent variables (joint configuration)
    ADVectorXs ad_q(model.nq);
    Independent(ad_q);

    // Compute forward kinematics
    pinocchio::forwardKinematics(ad_model, ad_data, ad_q);

    // Extract end-effector pose (assuming last frame is end-effector)
    // You can specify a specific frame index here
    FrameIndex end_effector_frame = ad_model.frames.size() - 1;

    // Update frame placements
    pinocchio::updateFramePlacements(ad_model, ad_data);

    // Get the transformation matrix
    const auto &oMf = ad_data.oMf[end_effector_frame];

    // Extract position and rotation matrix as dependent variables
    ADVectorXs ad_result(12);  // 3 for position + 9 for rotation matrix
    ad_result.head<3>() = oMf.translation();

    // Flatten rotation matrix
    const auto &R = oMf.rotation();
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            ad_result[3 + 3 * i + j] = R(i, j);
        }
    }

    // Create the automatic differentiation function
    ADFun<CGD> fk_function(ad_q, ad_result);

    // Generate C++ code
    CodeHandler<double> handler;
    CppAD::vector<CGD> indVars(model.nq);
    handler.makeVariables(indVars);

    // Evaluate function with symbolic variables
    CppAD::vector<CGD> result = fk_function.Forward(0, indVars);

    // Generate C++ code
    LanguageC<float> langC("float");
    LangCDefaultVariableNameGenerator<float> nameGen;

    std::ostringstream code;
    handler.generateCode(code, langC, result, nameGen);

    // Output the generated code
    std::cout << "\n=== Generated C++ Code for Forward Kinematics ===" << std::endl;
    std::cout << "#include <cmath>" << std::endl;
    std::cout << "\nvoid forward_kinematics(const double* q, double* result) {" << std::endl;
    std::cout << code.str() << std::endl;
    std::cout << "}" << std::endl;

    // Test the original function
    Eigen::VectorXd q_test = randomConfiguration(model);
    Data data(model);

    forwardKinematics(model, data, q_test);
    updateFramePlacements(model, data);

    std::cout << "\n=== Test with random configuration ===" << std::endl;
    std::cout << "Joint configuration: " << q_test.transpose() << std::endl;
    std::cout << "End-effector position: " << data.oMf[end_effector_frame].translation().transpose()
              << std::endl;

    return 0;
}
