cmake_minimum_required(VERSION 4.0)
project(Missile6DOF)

set(CMAKE_CXX_STANDARD 20)

add_library(sim_core
        src/core/State.cpp
        include/core/State.h
        src/MathTypes/Quaternion.cpp
        include/MathTypes/Quaternion.h
        src/MathTypes/Vec3.cpp
        include/MathTypes/Vec3.h
        include/MathTypes/MatrixXd.h
        include/ModelInterfaces/IAtmosphere.h
        src/dynamics/MissileDynamics.cpp
        include/dynamics/MissileDynamics.h
        src/MathTypes/Mat3.cpp
        include/MathTypes/Mat3.h
        include/ModelInterfaces/IAeroModel.h
        include/ModelInterfaces/IThrustModel.h
        include/ModelImplementations/ConstantAtmosphere.h
        include/ModelImplementations/ZeroAeroModel.h
        include/ModelImplementations/ZeroThrustModel.h
        include/ModelImplementations/LinearAeroModel.h
        src/core/Integrator.cpp
        include/core/Integrator.h
        src/core/Simulator.cpp
        include/core/Simulator.h
        include/ModelImplementations/ConstThrustModel.h
        include/ModelInterfaces/IMassProperties.h
        include/ModelInterfaces/IPropulsionModel.h
        include/ModelInterfaces/IActuatorModel.h
        src/dynamics/LoadsAggregator.cpp
        include/dynamics/LoadsAggregator.h
        include/ModelInterfaces/IGravityModel.h
        include/ModelImplementations/SimpleGimbaledPropulsionModel.h
        include/ModelImplementations/SimpleRocketMassPropertiesModel.h
        include/ModelImplementations/SimpleActuatorModel.h
        include/ModelImplementations/ZeroGravityModel.h
        src/logging/CSVLogger.cpp
        include/logging/CSVLogger.h
        include/logging/LogRecord.h
        include/ModelImplementations/ZeroPropulsionModel.h
        include/ModelImplementations/ConstantGravityModel.h
        include/GNCInterfaces/INavigationModel.h
        include/GNCImplementations/NavigationTruthModel.h
        include/GNCInterfaces/IGuidanceModel.h
        include/GNCImplementations/GuidanceAttitudeHold.h
        include/GNCInterfaces/IControllerModel.h
        include/GNCImplementations/PDAttitudeController.h
        include/GNCImplementations/ImuNavigation.h
        include/GNCImplementations/EKFNavigation.h
        include/sensors/IImuModel.h
        include/sensors/IdealImuModel.h
        include/sensors/BiasedImuModel.h
        include/sensors/NoisyImuModel.h
        include/sensors/IGpsModel.h
        include/sensors/IdealGpsModel.h
        include/sensors/NoisyGpsModel.h
)

target_include_directories(sim_core PUBLIC include)

#EXAMPLES

add_executable(attitude_hold src/examples/attitude_hold.cpp)
target_link_libraries(attitude_hold PRIVATE sim_core)

add_executable(imu_test src/examples/IMU_zero_thrust.cpp)
target_link_libraries(imu_test PUBLIC sim_core)

add_executable(imu_simple_thrust src/examples/IMU_simple_thrust.cpp)
target_link_libraries(imu_simple_thrust PUBLIC sim_core)

add_executable(imu_zero_torque src/examples/IMU_zero_torque.cpp)
target_link_libraries(imu_zero_torque PUBLIC sim_core)

add_executable(imu_accel_bias src/examples/IMU_accel_bias.cpp)
target_link_libraries(imu_accel_bias PUBLIC sim_core)

add_executable(linear_algebra src/examples/linear_algebra.cpp)
target_link_libraries(linear_algebra PUBLIC sim_core)

add_executable(ekf_inertial src/examples/EKF_inertial_coords.cpp)
target_link_libraries(ekf_inertial PUBLIC sim_core)

#TESTs

enable_testing()

add_executable(test_math src/testing/test_math.cpp)
target_link_libraries(test_math PRIVATE sim_core)
add_test(NAME test_math COMMAND test_math)

add_executable(test_actuators src/testing/test_actuators.cpp)
target_link_libraries(test_actuators PRIVATE sim_core)
add_test(NAME test_actuators COMMAND test_actuators)

add_executable(test_dynamics src/testing/test_dynamics.cpp)
target_link_libraries(test_dynamics PRIVATE sim_core)
add_test(NAME test_dynamics COMMAND test_dynamics)

add_executable(test_gnc src/testing/test_gnc.cpp)
target_link_libraries(test_gnc PRIVATE sim_core)
add_test(NAME test_gnc COMMAND test_gnc)

add_executable(test_matrix src/testing/test_MatrixXd.cpp)
target_link_libraries(test_matrix PRIVATE sim_core)
add_test(NAME test_matrix COMMAND test_matrix)