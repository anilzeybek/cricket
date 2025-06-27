import numpy as np
import pinocchio as pin
import pinocchio.cppadcg as cgpin
from pycppad import (
    ADCG,
    CG,
    ADCGFun,
    CodeHandler,
    Independent,
    LangCDefaultVariableNameGenerator,
    LanguageC,
)


def generate_forward_kinematics_code(urdf_filename, frame_name=None):
    """
    Generate C++ code for forward kinematics of a robot from URDF.
    
    Args:
        urdf_filename: Path to the URDF file
        frame_name: Name of the frame to compute FK for (if None, uses last frame)
    
    Returns:
        Generated C++ code as string
    """

    # Load the model
    model = pin.buildModelFromUrdf(urdf_filename)
    print(f"Model loaded with {model.nq} DOF")

    # Convert to code generation model
    cg_model = cgpin.Model(model)
    cg_data = cg_model.createData()

    nq = model.nq

    # Define independent variables (joint positions)
    x = np.array([ADCG(CG(0.0))] * nq)
    Independent(x)

    # Compute forward kinematics
    cgpin.forwardKinematics(cg_model, cg_data, x)
    cgpin.updateFramePlacements(cg_model, cg_data)

    # Get frame index
    if frame_name is None:
        frame_id = len(model.frames) - 1  # Last frame
        frame_name = model.frames[frame_id].name
    else:
        frame_id = model.getFrameId(frame_name)

    print(f"Generating code for frame: {frame_name}")

    # Extract transformation matrix
    oMf = cg_data.oMf[frame_id]

    # Create output vector: [x, y, z, R00, R01, R02, R10, R11, R12, R20, R21, R22]
    y = []

    # Position
    for i in range(3):
        y.append(oMf.translation[i])

    # Rotation matrix (flattened)
    for i in range(3):
        for j in range(3):
            y.append(oMf.rotation[i, j])

    # Create the automatic differentiation function
    y = np.array(y)
    fun = ADCGFun(x, y)

    # Generate C++ code
    handler = CodeHandler(100)
    indVars = np.array([CG(1.0)] * nq)
    handler.makeVariables(indVars)

    # Evaluate function
    result = fun.Forward(indVars)

    # Generate code
    langC = LanguageC("double", 4)  # Use 4 decimal precision
    nameGen = LangCDefaultVariableNameGenerator("result", "q", "v", "array",
                                                "sarray")
    code = handler.generateCode(langC, result, nameGen, "source")

    return code, frame_name


def generate_forward_kinematics_with_jacobian(urdf_filename, frame_name=None):
    """
    Generate C++ code for both forward kinematics and its Jacobian.
    """

    # Load the model
    model = pin.buildModelFromUrdf(urdf_filename)
    cg_model = cgpin.Model(model)
    cg_data = cg_model.createData()

    nq = model.nq

    # Define independent variables
    x = np.array([ADCG(CG(0.0))] * nq)
    Independent(x)

    # Compute forward kinematics
    cgpin.forwardKinematics(cg_model, cg_data, x)
    cgpin.updateFramePlacements(cg_model, cg_data)

    # Get frame
    if frame_name is None:
        frame_id = len(model.frames) - 1
    else:
        frame_id = model.getFrameId(frame_name)

    # Extract position only for simplicity
    oMf = cg_data.oMf[frame_id]
    y = np.array([oMf.translation[i] for i in range(3)])

    # Create function
    fun = ADCGFun(x, y)

    # Generate code for function and Jacobian
    handler = CodeHandler(100)
    indVars = np.array([CG(1.0)] * nq)
    handler.makeVariables(indVars)

    # Function evaluation
    result = fun.Forward(indVars)

    # Jacobian evaluation
    jac = fun.Jacobian(indVars)

    # Generate code
    langC = LanguageC("double", 4)
    nameGen = LangCDefaultVariableNameGenerator("y", "q", "v", "array",
                                                "sarray")

    func_code = handler.generateCode(langC, result, nameGen, "source")
    jac_code = handler.generateCode(langC, jac, nameGen, "source")

    return func_code, jac_code


# Example usage
if __name__ == "__main__":
    urdf_file = "panda/panda_spherized.urdf"  # Replace with actual path

    try:
        # Generate forward kinematics code
        fk_code, frame_name = generate_forward_kinematics_code(urdf_file)

        print(
            f"=== Generated C++ Code for Forward Kinematics of {frame_name} ==="
        )
        print("#include <cmath>")
        print()
        print(
            "// Function signature: void forward_kinematics(const double* q, double* result)"
        )
        print("// result[0:2] = position (x,y,z)")
        print("// result[3:11] = rotation matrix (row-major)")
        print("void forward_kinematics(const double* q, double* result) {")
        print(fk_code)
        print("}")

        # Generate Jacobian code as well
        print("\n=== Generating Jacobian Code ===")
        func_code, jac_code = generate_forward_kinematics_with_jacobian(
            urdf_file)

        print("// Position forward kinematics")
        print("void fk_position(const double* q, double* position) {")
        print(func_code)
        print("}")

        print("\n// Position Jacobian (3x" +
              str(pin.buildModelFromUrdf(urdf_file).nq) + ")")
        print("void fk_jacobian(const double* q, double* jacobian) {")
        print(jac_code)
        print("}")

    except Exception as e:
        print(f"Error: {e}")
        print("Make sure you have installed:")
        print("- pip install pycppad")
        print("- Pinocchio compiled with CppADCodeGen support")
