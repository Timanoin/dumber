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

#ifndef __TASKS_H__
#define __TASKS_H__

#include <unistd.h>
#include <iostream>

#include <sys/mman.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/mutex.h>
#include <alchemy/sem.h>
#include <alchemy/queue.h>

#include "messages.h"
#include "commonitor.h"
#include "comrobot.h"
#include "camera.h"
#include "img.h"

using namespace std;

class Tasks {
public:
    /**
     * @brief Initializes main structures (semaphores, tasks, mutex, etc.)
     */
    void Init();

    /**
     * @brief Starts tasks
     */
    void Run();

    /**
     * @brief Stops tasks
     */
    void Stop();
    
    /**
     * @brief Suspends main thread
     */
    void Join();
    
private:
    /**********************************************************************/
    /* Shared data                                                        */
    /**********************************************************************/
    ComMonitor monitor;
    ComRobot robot;
    int robotStarted = 0;
    int move = MESSAGE_ROBOT_STOP;
    // INSA
    int cpt = 0;
    Camera* camera;
    bool sendingImage;
    Arena* arena;
    Arena* tmp_arena;
    // END INSA
    
    /**********************************************************************/
    /* Tasks                                                              */
    /**********************************************************************/
    RT_TASK th_server;
    RT_TASK th_sendToMon;
    RT_TASK th_receiveFromMon;
    RT_TASK th_openComRobot;
    RT_TASK th_startRobot;
    RT_TASK th_move;

    // INSA Custom tasks
    RT_TASK th_updateBatteryLevel;
    RT_TASK th_startRobotWD;
    RT_TASK th_reloadWD;
    RT_TASK th_openCamera;
    RT_TASK th_cameraSendImage;
    RT_TASK th_closeCamera;
    RT_TASK th_findArena;
    // INSA End custom tasks
    
    /**********************************************************************/
    /* Mutex                                                              */
    /**********************************************************************/
    RT_MUTEX mutex_monitor;
    RT_MUTEX mutex_robot;
    RT_MUTEX mutex_robotStarted;
    RT_MUTEX mutex_move;

    // INSA custom mutex
    RT_MUTEX mutex_camera;
    // END INSA

    /**********************************************************************/
    /* Semaphores                                                         */
    /**********************************************************************/
    RT_SEM sem_barrier;
    RT_SEM sem_openComRobot;
    RT_SEM sem_serverOk;
    RT_SEM sem_startRobot;

    // INSA Cutom Semaphores
    RT_SEM sem_startRobotWD;
    RT_SEM sem_closeCamera;
    RT_SEM sem_openCamera;
    RT_SEM sem_findArena;
    // END INSA

    /**********************************************************************/
    /* Message queues                                                     */
    /**********************************************************************/
    int MSG_QUEUE_SIZE;
    RT_QUEUE q_messageToMon;
    
    /**********************************************************************/
    /* Tasks' functions                                                   */
    /**********************************************************************/
    /**
     * @brief Thread handling server communication with the monitor.
     */
    void ServerTask(void *arg);
     
    /**
     * @brief Thread sending data to monitor.
     */
    void SendToMonTask(void *arg);
        
    /**
     * @brief Thread receiving data from monitor.
     */
    void ReceiveFromMonTask(void *arg);
    
    /**
     * @brief Thread opening communication with the robot.
     */
    void OpenComRobot(void *arg);

    /**
     * @brief Thread starting the communication with the robot.
     */
    void StartRobotTask(void *arg);
    
    /**
     * @brief Thread handling control of the robot.
     */
    void MoveTask(void *arg);
    
    /**********************************************************************/
    /* Queue services                                                     */
    /**********************************************************************/
    /**
     * Write a message in a given queue
     * @param queue Queue identifier
     * @param msg Message to be stored
     */
    void WriteInQueue(RT_QUEUE *queue, Message *msg);
    
    /**
     * Read a message from a given queue, block if empty
     * @param queue Queue identifier
     * @return Message read
     */
    Message *ReadInQueue(RT_QUEUE *queue);

    // INSA

    // Feature 13
    // Display battery level
    void UpdateBatteryLevel(void *arg);

    // Feature 11 
    // Task that starts the robot with the watchdog on
    void StartRobotTaskWD(void *arg);

    // Feature 11 
    // Periodic task of period 1s that reloads the watchdog counter
    void ReloadWD(void *arg);

    // Features 8&9
    // Get message from the robot when writing and manages a 3 counter
    void checkWriteError(Message* msg);

    // Feature 14
    // Task that turns the camera on
    void OpenCamera(void *args);

    // Feature 15
    // Task that makes the camera an image to the monitor
    void CameraSendImage(void *args);

    // Feature 16
    // Task that turns the camera off   
    void CloseCamera(void *args);

    // Feature 17 
    // Task that tries to find the arena
    void FindArena(void *args);

    // END INSA

};

#endif // __TASKS_H__ 

