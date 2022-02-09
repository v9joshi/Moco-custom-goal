/* -------------------------------------------------------------------------- *
 *WalkSim_Old.cpp                                                           *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Author(s): Varun Joshi                                                     *
 *                                                                            *
 * -------------------------------------------------------------------------- */

#include <string.h>

#include "MocoActivationSquaredGoal.h"
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Moco/MocoGoal/MocoOutputGoal.h>

using namespace OpenSim;
using SimTK::Pi;

void young_sim(double targetSpeed) {

    std::cout << "target speed = " << targetSpeed << std::endl;
    std::string speedStr = std::to_string(targetSpeed);

    MocoStudy study;
    study.setName("Walksim_Young");

    // Model (dynamics).
    // -----------------
    Model model("../walk2D_young.osim");// _breakdown.osim");
    model.initSystem();

    // Weird roundabout way

    // Add metabolics
    Bhargava2004SmoothedMuscleMetabolics* metabolics =
        new Bhargava2004SmoothedMuscleMetabolics();
    metabolics->setName("metabolics");
    metabolics->set_use_smoothing(true);

    metabolics->addMuscle("hamstrings_r", model.getComponent<Muscle>("forceset/hamstrings_r"), 0.48999999999999999, 250000, 1.7323960434887626);
    metabolics->addMuscle("hamstrings_l", model.getComponent<Muscle>("forceset/hamstrings_l"), 0.48999999999999999, 250000, 1.7323960434887626);

    metabolics->addMuscle("bifemsh_r", model.getComponent<Muscle>("forceset/bifemsh_r"), 0.53000000000000003, 250000, 0.27435006331569201);
    metabolics->addMuscle("bifemsh_l", model.getComponent<Muscle>("forceset/bifemsh_l"), 0.53000000000000003, 250000, 0.27435006331569201);

    metabolics->addMuscle("glut_max_r", model.getComponent<Muscle>("forceset/glut_max_r"), 0.55000000000000004, 250000, 2.3223027516598802);
    metabolics->addMuscle("glut_max_l", model.getComponent<Muscle>("forceset/glut_max_l"), 0.55000000000000004, 250000, 2.3223027516598802);

    metabolics->addMuscle("iliopsoas_r", model.getComponent<Muscle>("forceset/iliopsoas_r"), 0.5, 250000, 1.2439005670792906);
    metabolics->addMuscle("iliopsoas_l", model.getComponent<Muscle>("forceset/iliopsoas_l"), 0.5, 250000, 1.2439005670792906);

    metabolics->addMuscle("rect_fem_r", model.getComponent<Muscle>("forceset/rect_fem_r"), 0.39000000000000001, 250000, 0.72948648328115995);
    metabolics->addMuscle("rect_fem_l", model.getComponent<Muscle>("forceset/rect_fem_l"), 0.39000000000000001, 250000, 0.72948648328115995);

    metabolics->addMuscle("vasti_r", model.getComponent<Muscle>("forceset/vasti_r"), 0.5, 250000, 4.1797445876163364);
    metabolics->addMuscle("vasti_l", model.getComponent<Muscle>("forceset/vasti_l"), 0.5, 250000, 4.1797445876163364);

    metabolics->addMuscle("gastroc_r", model.getComponent<Muscle>("forceset/gastroc_r"), 0.54000000000000004, 250000, 1.1115350360849832);
    metabolics->addMuscle("gastroc_l", model.getComponent<Muscle>("forceset/gastroc_l"), 0.54000000000000004, 250000, 1.1115350360849832);

    metabolics->addMuscle("soleus_r", model.getComponent<Muscle>("forceset/soleus_r"), 0.80000000000000004, 250000, 1.1840644591072);
    metabolics->addMuscle("soleus_l", model.getComponent<Muscle>("forceset/soleus_l"), 0.80000000000000004, 250000, 1.1840644591072);

    metabolics->addMuscle("dorsiflexors_r", model.getComponent<Muscle>("forceset/dorsiflexors_r"), 0.69999999999999996, 250000, 0.64233920074661122);
    metabolics->addMuscle("dorsiflexors_l", model.getComponent<Muscle>("forceset/dorsiflexors_l"), 0.69999999999999996, 250000, 0.64233920074661122);

    model.addComponent(metabolics);
    model.finalizeConnections();

    model.print("temp_young.osim");
    // normal stuff continues
    //model.initSystem();
    //model.finalizeConnections();

    // Define the optimal control problem.
// ===================================
    MocoProblem& problem = study.updProblem();
    ModelProcessor modelProcessor = ModelProcessor(model);

    problem.setModelProcessor(modelProcessor);

    std::cout << "adding goals" << std::endl;

    // Symmetry.
    auto* symmetryGoal = problem.addGoal<MocoPeriodicityGoal>("symmetryGoal");
    model.initSystem();

    // Symmetric coordinate values (except for pelvis_tx) and speeds.
    for (const auto& coord : model.getComponentList<Coordinate>()) {
        if (IO::EndsWith(coord.getName(), "_r")) {
            symmetryGoal->addStatePair({ coord.getStateVariableNames()[0],
                   std::regex_replace(coord.getStateVariableNames()[0],
                          std::regex("_r"), "_l") });
            symmetryGoal->addStatePair({ coord.getStateVariableNames()[1],
                    std::regex_replace(coord.getStateVariableNames()[1],
                            std::regex("_r"), "_l") });
        }
        if (IO::EndsWith(coord.getName(), "_l")) {
            symmetryGoal->addStatePair({ coord.getStateVariableNames()[0],
                    std::regex_replace(coord.getStateVariableNames()[0],
                            std::regex("_l"), "_r") });
            symmetryGoal->addStatePair({ coord.getStateVariableNames()[1],
                    std::regex_replace(coord.getStateVariableNames()[1],
                            std::regex("_l"), "_r") });
        }
        if (!IO::EndsWith(coord.getName(), "_l") &&
            !IO::EndsWith(coord.getName(), "_r") &&
            !IO::EndsWith(coord.getName(), "_tx")) {
            symmetryGoal->addStatePair({ coord.getStateVariableNames()[0],
                    coord.getStateVariableNames()[0] });
            symmetryGoal->addStatePair({ coord.getStateVariableNames()[1],
                    coord.getStateVariableNames()[1] });
        }
    }
    symmetryGoal->addStatePair({ "/jointset/ground_pelvis/pelvis_tx/speed" });

    // Symmetric muscle activations.
    for (const auto& muscle : model.getComponentList<Muscle>()) {
        if (IO::EndsWith(muscle.getName(), "_r")) {
            symmetryGoal->addStatePair({ muscle.getStateVariableNames()[0],
                    std::regex_replace(muscle.getStateVariableNames()[0],
                            std::regex("_r"), "_l") });
        }
        if (IO::EndsWith(muscle.getName(), "_l")) {
            symmetryGoal->addStatePair({ muscle.getStateVariableNames()[0],
                    std::regex_replace(muscle.getStateVariableNames()[0],
                            std::regex("_l"), "_r") });
        }
    }

    // Symmetric tendon forces.
    for (const auto& muscle : model.getComponentList<Muscle>()) {
        if (IO::EndsWith(muscle.getName(), "_r")) {
            symmetryGoal->addStatePair({ muscle.getStateVariableNames()[1],
                    std::regex_replace(muscle.getStateVariableNames()[1],
                            std::regex("_r"), "_l") });
        }
        if (IO::EndsWith(muscle.getName(), "_l")) {
            symmetryGoal->addStatePair({ muscle.getStateVariableNames()[1],
                    std::regex_replace(muscle.getStateVariableNames()[1],
                            std::regex("_l"), "_r") });
        }
    }

    // Symmetric controls.
    for (const auto& muscle : problem.createRep().createControlInfoNames()) {
        if (IO::EndsWith(muscle, "_r")) {
            symmetryGoal->addControlPair(MocoPeriodicityGoalPair(muscle, std::regex_replace(muscle, std::regex("_r"), "_l")));
        }
        if (IO::EndsWith(muscle, "_l")) {
            symmetryGoal->addControlPair(MocoPeriodicityGoalPair(muscle, std::regex_replace(muscle, std::regex("_l"), "_r")));
        }
    }

    // Effort. Get a reference to the MocoControlGoal that is added to every
    // MocoTrack problem by default.
    MocoControlGoal* effort = problem.addGoal<MocoControlGoal>("control_effort");
    effort->setWeight(0.001);
    effort->setDivideByDisplacement(true);
    effort->setExponent(2);


    // Metabolics; total metabolic rate includes activation heat rate,
    // maintenance heat rate, shortening heat rate, mechanical work rate, and
    // basal metabolic rate.
    auto* metGoal = problem.addGoal<MocoOutputGoal>("metcost", 1);
    //metGoal->setOutputPath("/smooth_metabolic_cost_young|total_metabolic_rate");
    metGoal->setOutputPath("/metabolics|total_metabolic_rate");
    metGoal->setDivideByDisplacement(true);
    metGoal->setDivideByMass(true);

    // Use the goal from the plugin:
    auto* actGoal = problem.addGoal<MocoActivationSquaredGoal>("ActivationCost", 10);
    actGoal->setDivideByDisplacement(true);

    // Add a speed goal:
    auto* speedGoal = problem.addGoal<MocoAverageSpeedGoal>();
    speedGoal->set_desired_average_speed(targetSpeed);

    std::cout << "adding bounds" << std::endl;

    // Add some bounds
    problem.setTimeBounds(0, { 0.2, 2.0 });

    problem.setStateInfo("/jointset/ground_pelvis/pelvis_tilt/value", { -20 * Pi / 180, -10 * Pi / 180 });
    problem.setStateInfo("/jointset/ground_pelvis/pelvis_tx/value", { -1, 2 }, 0, { 0.2,2.0 });
    problem.setStateInfo("/jointset/ground_pelvis/pelvis_ty/value", { 0.75, 1.25 });

    problem.setStateInfo("/jointset/knee_l/knee_angle_l/value", { -80 * Pi / 180, 10 * Pi / 180 });
    problem.setStateInfo("/jointset/knee_r/knee_angle_r/value", { -80 * Pi / 180, 10 * Pi / 180 });
    problem.setStateInfo("/jointset/ankle_l/ankle_angle_l/value",
        { -35 * Pi / 180, 25 * Pi / 180 });
    problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/value",
        { -35 * Pi / 180, 25 * Pi / 180 });
    problem.setStateInfo("/jointset/mtp_r/mtp_angle_r/value", { -25 * Pi / 180, 75 * Pi / 180 });
    problem.setStateInfo("/jointset/mtp_l/mtp_angle_l/value", { -25 * Pi / 180, 75 * Pi / 180 });

    // Young people hip bounds
    problem.setStateInfo("/jointset/hip_l/hip_flexion_l/value",
        { -20 * Pi / 180, 60 * Pi / 180 });
    problem.setStateInfo("/jointset/hip_r/hip_flexion_r/value",
        { -20 * Pi / 180, 60 * Pi / 180 });

    // Activation, force and control bounds
    problem.setStateInfoPattern("/forceset/.*/normalized_tendon_force", { 0,1.5 });
    problem.setStateInfoPattern("/forceset/.*/activation", { 0.01,1.0 });
    problem.setControlInfoPattern(".*", { 0.01, 1.0 });

    std::cout << "configuring solver" << std::endl;

    // Configure the solver.
    // =====================
    auto& solver = study.initCasADiSolver();

    std::cout << "changing solver settings" << std::endl;

    solver.set_num_mesh_intervals(25);
    solver.set_verbosity(2);
    solver.set_optim_solver("ipopt");
    //solver.set_multibody_dynamics_mode("implicit");
    solver.set_optim_max_iterations(2000);
    solver.set_optim_convergence_tolerance(0.001);
    solver.set_optim_constraint_tolerance(0.001);
    solver.set_parallel(12);
    solver.set_minimize_implicit_auxiliary_derivatives(true);
    solver.set_implicit_auxiliary_derivatives_weight(0.001);

    std::cout << "setting guess file" << std::endl;
    // Specify an initial guess.
    // -------------------------
    solver.setGuessFile("../gaitTracking_solution_050intervals.sto");

    std::cout << "solving study" << std::endl;
    // Print it out somewhere
    study.print("WalkSim_young.omoco");

    // Solve the problem.
    // ==================
    MocoSolution solution = study.solve();

    std::cout << "Solution status: " << solution.getStatus() << std::endl;
    if (solution.isSealed()) {
        solution.unseal();
    }

    std::cout << "Solution status: " << solution.getStatus() << std::endl;

    std::string fileHeader = "../Results/Young Healthy/cpp_young_";
    fileHeader = fileHeader + speedStr;
    std::string outputFileName = "_Solution.sto";
    outputFileName = fileHeader + outputFileName;
    //outputFileName = std::strcat(outputFileName, speedStr);
    std::string outputFileName_Full = "_fullStride.sto";
    outputFileName_Full = fileHeader + outputFileName_Full;
    //std::strcat(outputFileName, "_outputFull.sto");
    //outputFileName = std::strcat(outputFileName, +"_output.sto");
    
    solution.write(outputFileName);
    //solution.write("C:/Users/varunjos/Documents/GitHub/Walking/WalkSim/Reference_Solutions/recentbest_young.sto");

    MocoTrajectory fullStrideSol = createPeriodicTrajectory(solution);
    fullStrideSol.write(outputFileName_Full);

    std::cout << "Control effort: " << solution.getObjectiveTerm("control_effort") << std::endl;
    std::cout << "Metabolic cost: " << solution.getObjectiveTerm("metcost") << std::endl;
    std::cout << "Activation cost: " << solution.getObjectiveTerm("ActivationCost") << std::endl;

    std::vector<std::string> contact_r;
    std::vector<std::string> contact_l;
    contact_r.push_back("Contact_Foot_Ground_R1");
    contact_r.push_back("Contact_Foot_Ground_R2");
    contact_r.push_back("Contact_Foot_Ground_R3");
    contact_r.push_back("Contact_Foot_Ground_R4");
    contact_r.push_back("Contact_Foot_Ground_R5");
    contact_r.push_back("Contact_Foot_Ground_R6");
    contact_r.push_back("Contact_Foot_Ground_R7");
    contact_r.push_back("Contact_Foot_Ground_R8");

    contact_l.push_back("Contact_Foot_Ground_L1");
    contact_l.push_back("Contact_Foot_Ground_L2");
    contact_l.push_back("Contact_Foot_Ground_L3");
    contact_l.push_back("Contact_Foot_Ground_L4");
    contact_l.push_back("Contact_Foot_Ground_L5");
    contact_l.push_back("Contact_Foot_Ground_L6");
    contact_l.push_back("Contact_Foot_Ground_L7");
    contact_l.push_back("Contact_Foot_Ground_L8");

    std::string grfFileName = "_GRF.sto";
    grfFileName = fileHeader + grfFileName;
    TimeSeriesTable externalForcesTableFlat = createExternalLoadsTableForGait(model, fullStrideSol, contact_r, contact_l);
    STOFileAdapter::write(externalForcesTableFlat, grfFileName);

    study.visualize(solution);       

    //return EXIT_SUCCESS;
}

int main() {

    double speed = 0.6;
    young_sim(speed);

    return 1;
}