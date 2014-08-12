#include <iostream>

#include <kuros.h>

#include <HandlingServer.hpp>



void sleep(const int ms)
{
    std::this_thread::sleep_for( std::chrono::milliseconds(ms) );
}

int main()
{
    HandlingServer aserver;
    aserver.startListening();   // blocks until connection

    info_vec info
    {
        KUKA_RMODE_STREAM,  // response mode
        12,                 // [ms] response stream interval
        0,                  // a trajectory id
        YES,                // NO robot exits after trajectory, YES robot keeps running
        200,                // [mm/sec], comfortable vel = 200
        20,                  // [mm*0.01], max approximation distance from trajectory point
        KUKA_CARTESIAN      // KUKA_CARTESIAN X Y Z Y P R, KUKA_AXIS A1 A2 A3 A4 A5 A6 (not yet supported)
    };

    double step = 1;    // [mm]

    frame_vec down {0,0,-step,0,0,0};
    frame_vec up {0,0,step,0,0,0};
    frame_vec dontmove {0,0,0,0,0,0};
    frame_vec left {0,-step,0,0,0,0};
    frame_vec right {0,step,0,0,0,0};
    frame_vec towall {-step,0,0,0,0,0};
    frame_vec fromwall {step,0,0,0,0,0};

    //std::vector<std::string> com {"stop", "down", "up", "left", "right", "to wall", "from wall" };
    std::vector<std::string> com {"down", "up", "stop"};

    // we use this to toggle between corrections
    //std::vector<frame_vec> corrections {dontmove, down, up, left, right, towall, fromwall};
    std::vector<frame_vec> corrections {down, up, dontmove};

    int corrsize = boost::lexical_cast<int>(corrections.size());
    int counter = 0;

    cout << "Starting to send corrections with stepsize " << step << " mm." << endl;

    aserver.startCapturing();

    while (aserver.isAccepting())
    {
        // every n seconds, change direction
        sleep(1000);

        int next = counter % corrsize;

        aserver.currentDecision = next; // "the moment of decisison"

        aserver.sendTrajectory(info, trajectory_vec{corrections[next]} );
        cout <<  com[next] << endl;

        if (counter == 10)  // lets capture so many decisions
        {
            aserver.finishCapturing();
        }

        ++counter;
    }

    cout << "Main thread exiting." << endl;
    return 0;

}
