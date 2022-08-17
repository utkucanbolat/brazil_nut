#include "Mercury3D.h"
#include "Species/Species.h"
#include "Walls/InfiniteWall.h"
#include "Boundaries/CubeInsertionBoundary.h"
#include "Boundaries/DeletionBoundary.h"
#include "Species/LinearViscoelasticFrictionSpecies.h"
#include "Walls/AxisymmetricIntersectionOfWalls.h"

// Global variables
double big_particle_radius = 0.08;
double small_particle_radius = 0.02;
double eps = 0.2;  // eps: the opening of the cube insertion boundary.

double stop_flow = 1.0;  // the time-step that the insertion boundary stops
double oscillation_amplitude = 1.0;  // the amplitude of the velocity. the higher this value the more the wall moves
double pulse_interval = 0.25;  // the interval between velocity change
double stop_kick = 20.0; // after this time-step the shake stops

class Quantum : public Mercury3D {
public:
    void setupInitialConditions() override {
        // Determine the dimension of the cylinder
        Vec3D mid = {
                (getXMin() + getXMax()) / 2.0,
                (getYMin() + getYMax()) / 2.0,
                (getZMin() + getZMax()) / 2.0};

        // Put cylindrical container
        AxisymmetricIntersectionOfWalls w;
        w.setSpecies(speciesHandler.getObject(0));
        w.setPosition(Vec3D(mid.X, mid.Y, 0));
        w.setOrientation(Vec3D(0, 0, 1));
        w.addObject(Vec3D(1, 0, 0), Vec3D((getXMax() - getXMin()) / 4.0, 0, 0));  //Cylindric wall
        wallHandler.copyAndAddObject(w);

        // Add the big particle to the system
        SphericalParticle p1;
        p1.setSpecies(speciesHandler.getObject(0));
        p1.setRadius(big_particle_radius); // sets particle radius
        p1.setPosition(Vec3D(mid.X, mid.Y, 2*big_particle_radius)); // sets particle position
        p1.setVelocity(Vec3D(0, 0, 0));
        particleHandler.copyAndAddObject(p1);

        // Determine the specifications of the small particles
        SphericalParticle p0;
        p0.setSpecies(speciesHandler.getObject(0));
        p0.setVelocity(Vec3D(0, 0, 0));
        p0.setRadius(small_particle_radius); // sets particle radius

        // Put the insertion boundary to fill the container with small particles
        CubeInsertionBoundary ib;
        ib.set(p0, 1, Vec3D(mid.X - eps, mid.Y - eps, mid.Z - eps), Vec3D(mid.X + eps, mid.Y + eps, mid.Z + eps),
               Vec3D(0, 0, 0), Vec3D(0, 0, 0));
        ib.setVolumeFlowRate(1);
        boundaryHandler.copyAndAddObject(ib);

        // Add the top wall for preventing particles to escape
        InfiniteWall top_wall;
        top_wall.setSpecies(speciesHandler.getObject(0));
        top_wall.set(Vec3D(0.0, 0.0, 1.0), Vec3D(0.0, 0.0, getZMax()));
        wallHandler.copyAndAddObject(top_wall);

        // Add the bottom wall. Later we change its velocity to impose shake to the system
        InfiniteWall bottom_wall;
        bottom_wall.setSpecies(speciesHandler.getObject(0));
        bottom_wall.set(Vec3D(0.0, 0.0, -1.0), Vec3D(0.0, 0.0, 0.0));
        wallHandler.copyAndAddObject(bottom_wall);
    }

    void actionsAfterTimeStep() override {
        static int kick_count = 0;  // this parameter keeps track of how many kick is applied
        static double kick_interval_counter = 1.5;  // the time the wall starts moving

        if (getTime() < stop_flow && getTime() + getTimeStep() > stop_flow) {
            dynamic_cast<CubeInsertionBoundary*>(boundaryHandler.getLastObject())->setVolumeFlowRate(0);
        } else if (getTime() > kick_interval_counter && getTime() < kick_interval_counter + pulse_interval) {
            wallHandler.getLastObject()->setVelocity(Vec3D(0.0, 0.0, oscillation_amplitude * pow(-1, kick_count)));
            kick_interval_counter += pulse_interval;
            kick_count += 1;
        } else if (getTime() > stop_kick) {
            wallHandler.getLastObject()->setVelocity(Vec3D(0.0, 0.0, 0.0));
        }
    }
};


int main(int argc, char *argv[]) {

    // Instantiate the simulation parameters
    Quantum problem;
    problem.setName("brazil_nut_cylinder");
    problem.setSystemDimensions(3);
    problem.setParticleDimensions(3);
    problem.setGravity(Vec3D(0.0, 0.0, -9.8));
    problem.setXMax(1.0);
    problem.setYMax(1.0);
    problem.setZMax(3.0);
    problem.setTimeMax(25.0);


    LinearViscoelasticFrictionSpecies species;
    species.setDensity(2000);
    species.setStiffness(1e5);
    species.setDissipation(0.63);
    species.setSlidingFrictionCoefficient(0.5);
    species.setSlidingStiffness(1.2e4);
    species.setSlidingDissipation(0.16);
    species.setRollingFrictionCoefficient(0.2);
    species.setRollingStiffness(1.2e4);
    species.setRollingDissipation(6.3e-2);
    species.setTorsionFrictionCoefficient(0.1);
    species.setTorsionStiffness(1.2e4);
    species.setSlidingDissipation(6.3e-2);
    problem.speciesHandler.copyAndAddObject(species);


    problem.setSaveCount(100);
    problem.setWallsWriteVTK(true);
    problem.setParticlesWriteVTK(true);

    logger(INFO, "run number: %", problem.dataFile.getCounter());

    problem.setXBallsAdditionalArguments("-solidf -v0 -noborder 4 -cube");

    problem.setTimeStep(0.005 / 50.0); // (collision time)/50.0

    problem.setNumberOfOMPThreads(30);  // parallelization
    problem.solve(argc, argv);

    return 0;
}
