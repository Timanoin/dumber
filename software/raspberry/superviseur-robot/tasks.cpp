/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 30
#define PRIORITY_TMOVE 20
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 31
#define PRIORITY_TCAMERA 21


// INSA CONSTANTS
#define PRIORITY_TBATTERYLEVEL 30
#define PRIORITY_TSTARTROBOTWD 20
#define PRIORITY_TRELOADWD 31

#define PRIORITY_TOPENCAMERA 28
#define PRIORITY_TCAMERASENDIMAGE 21
#define PRIORITY_TCLOSECAMERA 28

#define PRIORITY_TFINDARENA 28
#define PRIORITY_TREQPOS 21
#define PRIORITY_TSTOPPOS 21
#define PRIORITY_TKILLCOMM 31

// END CONSTANTS

/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;

    // INSA
    camera = new Camera(1,5);
    // END INSA

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    // INSA MUTEX
    if (err = rt_mutex_create(&mutex_camera, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    // END INSA
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
 

    // INSA Custom semaphores

    // Watchdog(11)

    if (err = rt_sem_create(&sem_startRobotWD, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    // Close Camera(16)
    if (err = rt_sem_create(&sem_closeCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    // Open Camera (14)
    if (err = rt_sem_create(&sem_openCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    // Find Arena (17)
    if (err = rt_sem_create(&sem_findArena, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    // Request Position (18)
    if (err = rt_sem_create(&sem_reqPosition, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    // Stop Requesting Position (19)
    if (err = rt_sem_create(&sem_stopPosition, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    // Kill Communication (6)
    if (err = rt_sem_create(&sem_killComm, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    // End
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    // INSA Custom tasks

    // Battery Level (13)

    if (err = rt_task_create(&th_updateBatteryLevel, "th_updateBetteryLevel", 0, PRIORITY_TBATTERYLEVEL, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    // Watchdog (11)
    if (err = rt_task_create(&th_startRobotWD, "th_startRobotWD", 0, PRIORITY_TSTARTROBOTWD, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_reloadWD, "th_reloadWD", 0, PRIORITY_TRELOADWD, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    // Camera (14,15,16)
    if (err = rt_task_create(&th_openCamera, "th_openCamera", 0, PRIORITY_TOPENCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_closeCamera, "th_closeCamera", 0, PRIORITY_TCLOSECAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_cameraSendImage, "th_cameraSendImage", 0, PRIORITY_TCAMERASENDIMAGE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    // Arena (17)
    if (err = rt_task_create(&th_findArena, "th_findArena", 0, PRIORITY_TFINDARENA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    // Position (18,19)
    if (err = rt_task_create(&th_reqPosition, "th_reqPosition", 0, PRIORITY_TREQPOS, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_stopPosition, "th_stopPosition", 0, PRIORITY_TSTOPPOS, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    // Kill communication
    if (err = rt_task_create(&th_killComm, "th_killComm", 0, PRIORITY_TKILLCOMM, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    // END custom tasks

    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    // INSA Custom tasks

    // Battery level (13)

    if (err = rt_task_start(&th_updateBatteryLevel, (void(*)(void*)) & Tasks::UpdateBatteryLevel, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    // Start robot with Watchdog (11)

    if (err = rt_task_start(&th_startRobotWD, (void(*)(void*)) & Tasks::StartRobotTaskWD, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
        if (err = rt_task_start(&th_reloadWD, (void(*)(void*)) & Tasks::ReloadWD, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    // Camera management (14-15-16)
    if (err = rt_task_start(&th_openCamera, (void(*)(void*)) & Tasks::OpenCamera, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_cameraSendImage, (void(*)(void*)) & Tasks::CameraSendImage, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_closeCamera, (void(*)(void*)) & Tasks::CloseCamera, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    // Draw arena (17)
    if (err = rt_task_start(&th_findArena, (void(*)(void*)) & Tasks::FindArena, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    // Request position (18-19)
    if (err = rt_task_start(&th_reqPosition, (void(*)(void*)) & Tasks::RequestPosition, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_stopPosition, (void(*)(void*)) & Tasks::StopPosition, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    // Kill Communication (6)
    if (err = rt_task_start(&th_killComm, (void(*)(void*)) & Tasks::KillComm, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    // INSA End custom tasks
    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
    monitorClosed = false;
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            rt_sem_v(&sem_killComm);
            cout << "TESTESTETSTESTTETETES" << endl << flush;
            
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            rt_sem_v(&sem_startRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
        // INSA 
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)) {
            rt_sem_v(&sem_startRobotWD);
        } 
        else if (msgRcv->CompareID(MESSAGE_CAM_OPEN)) {
            rt_sem_v(&sem_openCamera);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_CLOSE)) {
            rt_sem_v(&sem_closeCamera);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)) {
            rt_sem_v(&sem_findArena);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)) {
            if (tmp_arena != nullptr) arena = tmp_arena;
            sendingImage = true;
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)) {
            arena = nullptr;
            sendingImage = true;
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START)) {
            rt_sem_v(&sem_reqPosition);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP)) {
            rt_sem_v(&sem_stopPosition);
        }

        // END INSA
         delete(msgRcv); // mus be deleted manually, no consumer
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {

        Message * msgSend;
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        cout << "Start robot without watchdog (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(robot.StartWithoutWD());
        rt_mutex_release(&mutex_robot);
        checkWriteError(msgSend);

        cout << msgSend->GetID();
        cout << ")" << endl << flush;

        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
        }
    }
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
            
            cout << " move: " << cpMove;
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            checkWriteError(robot.Write(new Message((MessageID)cpMove)));
            rt_mutex_release(&mutex_robot);
        }
        cout << endl << flush;
    }
}

/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}

// INSA Tasks

// Feature 13
// Display battery level
void Tasks::UpdateBatteryLevel(void *arg)
{
    Message* batteryLevel;
    int rs;
    // Block while resources arent ready
    rt_sem_p(&sem_barrier, TM_INFINITE);
    // Task of period 500ms
    rt_task_set_periodic(NULL, TM_NOW, 500000000);
    while (1) {
        // Wait for period
        rt_task_wait_period(NULL);
        // Check if the robot is started 
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            // Block robot resources
            rt_mutex_acquire(&mutex_robot, TM_INFINITE); 
            // Collecting data
            batteryLevel = (Message*)robot.Write(new Message(MESSAGE_ROBOT_BATTERY_GET));//robot.Write(robot.getBattery());
            checkWriteError(batteryLevel);
            // Release robot resources
            rt_mutex_release(&mutex_robot);  
            // Send message to monitor with battery level
            WriteInQueue(&q_messageToMon, batteryLevel); 
        }
    }
}

// Feature 11 
// Task that starts the robot with the watchdog on
void Tasks::StartRobotTaskWD(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {

        Message * msgSend;
        rt_sem_p(&sem_startRobotWD, TM_INFINITE);
        cout << "Start robot with watchdog (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(robot.StartWithWD());
        rt_mutex_release(&mutex_robot);
        checkWriteError(msgSend);

        cout << msgSend->GetID();
        cout << ")" << endl;

        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
        }
    }
}

// Feature 11 
// Periodic task of period 1s that reloads the watchdog counter
void Tasks::ReloadWD(void *arg) {
    int rs;
    // Block while resources arent ready
    rt_sem_p(&sem_barrier, TM_INFINITE);
    // Task of period 1s
    rt_task_set_periodic(NULL, TM_NOW, 1000000000);
    while (1) {
        // Wait for period
        rt_task_wait_period(NULL);
        // Check if the robot is started 
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            // Block robot resources
            rt_mutex_acquire(&mutex_robot, TM_INFINITE); 
            // Collecting data
            checkWriteError(robot.Write(robot.ReloadWD()));
            // Release robot resources
            rt_mutex_release(&mutex_robot);   
        }
    }
}

// Features 8&9
// Get message from the robot when writing and manages a 3 counter
void Tasks::checkWriteError(Message* msg)
{
    // Compteur a trois
    if (*msg == MESSAGE_ANSWER_ROBOT_TIMEOUT || *msg == MESSAGE_ANSWER_COM_ERROR)
    {
        cpt += 1;
        cout << endl << "/!\\ Compteur : " << cpt << endl << flush;
    }
    else 
    {
        cpt = 0;
    }
    // Eteindre le robot quand la communication est perdue
    if (cpt >= 3)
    {
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        robotStarted = 0;
        rt_mutex_release(&mutex_robotStarted);
    }    
}

// Feature 14
// Task that turns the camera on
void Tasks::OpenCamera(void *args)
{   
    bool co, success = 0;
    rt_sem_p(&sem_barrier, TM_INFINITE);
    while (1) {
        rt_sem_p(&sem_openCamera, TM_INFINITE);
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        co = camera->IsOpen();
        rt_mutex_release(&mutex_camera);
        if (!co)
        {
            cout << endl << "Opening camera" << endl << flush;
            // Check if the camera is started 
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            success = camera->Open();
            rt_mutex_release(&mutex_camera);
            if (success) {
                sendingImage = true;
                cout << endl << "Camera Open !" << endl << flush;
            }
            else {
                cout << endl << "Open camera failed" << endl << flush;
            }
        } else {
            cout << endl << "Camera is already open" << endl << flush;
        }
    }
}


// Feature 15
// Periodic task of period 100ms that sends an image from the camera to the monitor
void Tasks::CameraSendImage(void *args)
{
    bool co = 0; // camera open ?   
    // Block while resources arent ready
    rt_sem_p(&sem_barrier, TM_INFINITE);
    // Task of period 100ms
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    while (1) {
        // Wait for period
        rt_task_wait_period(NULL);
        // Check if the camera is started 
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        co = camera->IsOpen();
        rt_mutex_release(&mutex_camera);
        if (co && sendingImage) {
            // Grab an image from the camera
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            Img* image = new Img(camera->Grab());
            rt_mutex_release(&mutex_camera);
            // Add arena (17)
            if (arena != nullptr)
            {
                image->DrawArena(*arena);
                // Draw position of robots (18)
                if (sendingPosition)
                {
                    std::list<Position> list = image->SearchRobot(*arena);
                    for (std::list<Position>::iterator it = list.begin(); it != list.end(); it++)
                    {
                        image->DrawRobot(*it);
                        MessagePosition* msgpos = new MessagePosition(MESSAGE_CAM_POSITION, *it);
                        // Send message to monitor with position
                        WriteInQueue(&q_messageToMon, msgpos);
                    }
                }
            }
            // Send image to the monitor
            MessageImg* msgimg = new MessageImg(MESSAGE_CAM_IMAGE, image); 
            // Send message to monitor with image
            WriteInQueue(&q_messageToMon, msgimg);
            cout << endl << "Sending Image.........." << endl << flush;

        }
    }
}

// Feature 16
// Task that turns the camera off
void Tasks::CloseCamera(void *args)
{
    bool co = 0;
    rt_sem_p(&sem_barrier, TM_INFINITE);  
    while (1) {
        rt_sem_p(&sem_closeCamera, TM_INFINITE);
        cout << endl << "Closing camera" << endl << flush;
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        co = camera->IsOpen();
        rt_mutex_release(&mutex_camera);
        if (co){
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            camera->Close();
            co = camera->IsOpen();
            rt_mutex_release(&mutex_camera);
            if (co){
                cout << endl << "Camera not closed" << endl << flush;
            } else {
                sendingImage = false;
                cout << endl << "Camera closed" << endl << flush;
            }
        }
    }
}

// Feature 17 
// Task that tries to find the arena

void Tasks::FindArena(void *args)
{   
    bool co = 0;
    rt_sem_p(&sem_barrier, TM_INFINITE);
    while (1) {
        rt_sem_p(&sem_findArena, TM_INFINITE);
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
        co = camera->IsOpen();
        rt_mutex_release(&mutex_camera);
        if (co)
        {
            sendingImage = false;
            // Grab an image from the camera
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            Img* image = new Img(camera->Grab());
            rt_mutex_release(&mutex_camera);
            tmp_arena = new Arena(image->SearchArena());
            if (tmp_arena->IsEmpty() || tmp_arena == nullptr)
            {
                Message* msg_nack = new Message(MESSAGE_ANSWER_NACK); 
                WriteInQueue(&q_messageToMon, msg_nack);
            } else {
                // Draw the arena on the image
                image->DrawArena(*tmp_arena);
                // Send the image with the arena to ask for confirmation
                MessageImg* msgimg = new MessageImg(MESSAGE_CAM_IMAGE, image); 
                WriteInQueue(&q_messageToMon, msgimg);
            }
        } else {
            cout << endl << "Camera is not open" << endl << flush;
        }
    }
}

// Feature 18
// Task that turns the camera on
void Tasks::RequestPosition(void *args)
{   
    rt_sem_p(&sem_barrier, TM_INFINITE);
    while (1) {
        rt_sem_p(&sem_reqPosition, TM_INFINITE);
        if (!sendingPosition) sendingPosition = true;
        cout << endl << "Requesting robot position" << endl << flush;
    }
}

// Feature 19
// Task that stops the position
void Tasks::StopPosition(void *args)
{
    rt_sem_p(&sem_barrier, TM_INFINITE);
    while (1) {
        rt_sem_p(&sem_stopPosition, TM_INFINITE);
        if (sendingPosition) sendingPosition = false;
        cout << endl << "Stop requesting robot position" << endl << flush;
    }
}

// Feature 5&6
// Task that kills the comunication between the robot, the supervisor and the monitor
// It sends a message to the monitor
void Tasks::KillComm(void *args)
{
    rt_sem_p(&sem_barrier, TM_INFINITE);
    while (1) {
        rt_sem_p(&sem_killComm, TM_INFINITE);
        if (!monitorClosed)
        {
            // Message sent to terminal
            cout << endl << "/!\\ ERROR: communication with monitor lost." << endl << flush; 
            // Reset class attributes ("global variables")
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 0;
            rt_mutex_release(&mutex_robotStarted);
            move = MESSAGE_ROBOT_STOP;
            // Compteur à trois
            cpt = 0;
            // Blocks or unlocks the camera
            sendingImage = false;
            // Current arena drawn on screen
            arena = nullptr;
            // Temporary
            tmp_arena = nullptr;
            // Display or not the position of the robot
            sendingPosition = false;

            /* rt_mutex_acquire(&mutex_robot, TM_INFINITE);   

            // Stop robot  
            robot.Write(robot.Stop());
            robot.Write(robot.Reset());
            // Stop communication with robot
            robot.Close();

            rt_mutex_release(&mutex_robot); */

            // Close camera
            rt_sem_v(&sem_closeCamera);
            
            /* // Close server
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);   
            monitor.Close();
            rt_mutex_release(&mutex_monitor); */
            Stop();
            monitorClosed = true;
            
        }

                    
    }
}
// END INSA
