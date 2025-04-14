#pragma once

#include <thread>
#include <utility>
#include <bits/stdc++.h>
#include <stdexcept>

#include "Logger.h"
#include "TimeUtils.h"

// TODO switch to std::atomic 's for bools

/**
 * @brief An asynchronous worker class.
 *
 * Worker lifecycle
 *
 * Instantiation: create worker object with a thread name, stay_alive flag, desired execution frequency,and logging verbosity.
 *
 * Start(): Called once. Calls private run() function to begin thread execution.
 *
 * Stop(): Called once. Ends thread execution; blocks until the thread has stopped. If param interrupted=false AND stay_alive
 *       is true, then the worker is restarted. By default, stay_alive=true. A stopped worker cannot be restarted, a
 *       new object must be created.
 *
 * Init(): Called once when the worker is told to begin execution (after Start is called)
 *
 * Execute(): Called periodically at a maximum rate of _exec_freq Hz until Stop() is called.
 *
 * Finish(): Called once before the Worker is destroyed (before destructor).
 *
 * Join(): Blocks until the thread object has finished.
 *
 * run(): Private member function that handles Execute timing and monitors for flags to stop execution.
 *
 */
class Worker{

public:
    /**
     * @brief Default constructor with placeholder threadname.
     */
    Worker() : Worker("THREADNAME"){}

    /**
     * @brief A worker with a specified thread name.
     * @param thread_name The thread name of the worker.
     */
    explicit Worker(const std::string& thread_name){
        _thread_name = thread_name;
        _t_worker = nullptr;
    }
    /**
     * @brief A worker with specified thread name and stay_alive flag.
     * @param thread_name The thread name of the worker.
     * @param stay_alive Flag to restart worker when Stop(false) is called.
     */
    explicit Worker(const std::string& thread_name, bool stay_alive): Worker(thread_name) {
        _stay_alive = stay_alive;
    }
    /**
     * @brief A worker with specified thread name, stay_alive flag, and execution_frequency.
     * @param thread_name The thread name of the worker.
     * @param stay_alive Flag to restart worker when Stop(false) is called.
     * @param execution_freq The maximum execution frequency, in hertz.
     */
    explicit Worker(const std::string& thread_name, bool stay_alive, double execution_freq): Worker(thread_name, stay_alive){
        _exec_freq = execution_freq;
    }
    /**
     * @brief A worker with specified thread name, stay_alive flag, execution_frequency, and debug verbosity.
     * @param thread_name The thread name of the worker.
     * @param stay_alive Flag to restart worker when Stop(false) is called.
     * @param execution_freq The maximum execution frequency, in hertz.
     * @param debug_verbosity The minimum verbosity of events to log.
     */
    explicit Worker(const std::string& thread_name, bool stay_alive, double execution_freq, AppLogger::SEVERITY debug_verbosity): Worker(thread_name, stay_alive, execution_freq){
        _debug_v = debug_verbosity;
    }
    /**
     * @brief Copy constructor, do not copy the thread object.
     * @param o The object to copy from.
     */
    Worker(const Worker& o){
        _exec_freq = o._exec_freq;
        _thread_name = o._thread_name;
        _debug_v = o._debug_v;
        _stay_alive = o._stay_alive;
        _t_worker = nullptr;
    }
    /**
     * @brief Ensure the thread has stopped execution, then delete the thread.
     */
    ~Worker(){
        _stop_sem.acquire();

        if (!_stop){
            _stop_sem.release();
            Stop();
        } else{
            _stop_sem.release();
        }

        _t_worker->join();
        delete _t_worker;
    }
    /**
     * @brief Change the stay_alive flag for the worker. Can be modified before or after a Worker has been started.
     * @param stay_alive true to keep the worker alive after Stop(false) is called, false to kill the worker instead.
     */
    void SetStayAlive(bool stay_alive){
        _stay_alive_sem.acquire();
        _stay_alive = stay_alive;
        _stay_alive_sem.release();
    }
    /**
     * @brief Get the value for the stay_alive flag of the worker. true keeps the worker alive after Stop(false) is called,
     * false kills the worker instead.
     * @return The value of the stay_alive flag.
     */
    bool GetStayAlive(){
        bool stay_alive;
        _stay_alive_sem.acquire();
        stay_alive = _stay_alive;
        _stay_alive_sem.release();
        return stay_alive;
    }
    /**
     * @brief Check if this worker has stopped.
     * @return true if the worker has stopped, false otherwise.
     */
    bool Stopped(){
        bool stop;
        _stop_sem.acquire();
        stop = _stop;
        _stop_sem.release();
        return stop;
    }
    /**
     * @brief Get the name of the worker.
     * @return The worker name.
     */
    const std::string& GetName(){
        return _thread_name;
    }
    /**
     * @brief Start the worker.
     */
    void Start(){
        _t_worker = new std::thread([this]() {this->run();});
    };
    /**
     * @brief Stop the worker. If the worker has the stay_alive flag set to true, then Stop(false) will simply
     * call the Init() function again, then resume execution of the Execute() function. If the worker has the stay_alive
     * flag set to false OR Stop() is called, the worker will finish its loop in the run() function, then call Finish().
     * @param interrupted If (true) OR (false AND stay_alive==false), tell the worker to end execution,
     * then call Finished(). Otherwise the worker will end execution, call Init(), then resume execution.
     * @return If the worker will completely stop, return true. Otherwise return false.
     */
    bool Stop(bool interrupted=true){
        _stop_sem.acquire();
        _interrupted_sem.acquire();
        _stop = true;
        if (!_interrupted) _interrupted = interrupted; // only change interrupted flag if not already interrupted
        _stay_alive_sem.acquire();
        if (_stay_alive && !interrupted){
            _stay_alive_sem.release();
            _stop_sem.release();
            _interrupted_sem.release();
            return false;
        }
        _stop_sem.release();
        _interrupted_sem.release();
        _stay_alive_sem.release();
        while (!IsFinished()){
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
//            AppLogger::Logger::Log("in worker stop function, waiting for " + GetName() + " to finish, stop=" + std::to_string(_stop));
        }
        return true;
    };

    bool IsFinished(){
        _is_finished_sem.acquire();
        bool is_finished = _is_finished;
        _is_finished_sem.release();
        return is_finished;
    }

    /**
     * @brief Block until the worker finishes execution.
     */
    void Join(){
        if (_t_worker){
            _t_worker->join();
        }
    };
    /**
     * @brief Get the actual execution frequency in hertz.
     * @return The actual execution frequency in hertz.
     */
    double GetExecutionFreq(){
        double ret_exec_freq;
        if (_exec_freq_sem.try_acquire_for(std::chrono::duration<ulong, std::milli>(100))){
            ret_exec_freq = _measure_exec_freq;
            _exec_freq_sem.release();
        }

        return ret_exec_freq;
    }
    /**
     * @brief Set the desired execution frequency in hertz. Blocks until the frequency is set.
     * @param freq The desired execution frequency in hertz.
     */

    void SetExecutionFreq(double freq){
        _exec_freq_sem.acquire();
        _exec_freq = freq;
        _exec_freq_sem.release();
    }

protected:
    /**
     * @brief The user-defined Execute function runs once per execution_freq. The execute function should never block. No
     * timeouts are implemented to detect loop overrun.
     */
    virtual void Execute() {};
    /**
     * @brief The user-defined Init function runs once, right after Start() is called. This function should never block. No
     * timeouts are implemented to detect loop overrun.
     */
    virtual void Init() {};
    /**
     * @brief The user-defined Finish function runs once, after Stop(true) is called OR (Stop(false) AND stay_alive == false).
     * This function should never block. No timeouts are implemented to detect loop overrun.
     */
    virtual void Finish() {
        _is_finished_sem.acquire();
        _is_finished = true;
        _is_finished_sem.release();
        AppLogger::Logger::Log(_thread_name + " exited");
    };
    std::thread* _t_worker{};

private:
    bool _stop = false;
    std::binary_semaphore _stop_sem{1};
    bool _stay_alive = true;
    std::binary_semaphore _stay_alive_sem{1};
    bool _interrupted = false;
    std::binary_semaphore _interrupted_sem{1};
    double _exec_freq = 100.0;

    bool _is_finished = false;
    std::binary_semaphore _is_finished_sem{1};

    double _measure_exec_freq{};
    std::binary_semaphore _exec_freq_sem{1};

    std::string _thread_name;
    int _debug_v = AppLogger::SEVERITY::INFO;

    /**
     * @brief The function that the worker's thread executes. First, run Init(), then run Execute() function at a maximum
     * rate of _exec_freq. No timeout if execution cannot match desired frequency. Periodically check flags to determine
     * if thread should stop. If thread should stop, run the Finish() function, then exit.
     */
    void run(){
        while (true) {
            try {

                std::vector<ulong> runtimes;
                ulong last_loop_ns, last_log_time_ns, current_duration_ns;
                last_loop_ns = CurrentTime();
                last_log_time_ns = last_loop_ns;

                Init();

                while (true) {

                    _stop_sem.acquire();
                    if (_stop) {
                        _stop_sem.release();
                        break;
                    }
                    _stop_sem.release();

                    current_duration_ns = CurrentTime() - last_loop_ns;
                    if (current_duration_ns >= ((1.0 / _exec_freq) * 1.0e9)) {
                        last_loop_ns = CurrentTime();

                            Execute();

//                        });
//
//                        if (future.wait_for(std::chrono::milliseconds(100)) == std::future_status::timeout) {
//                            AppLogger::Logger::Log(_thread_name + " Execute() timeout!", AppLogger::SEVERITY::WARNING);
//                        } else {
//                            future.get(); // Get any exceptions thrown by Execute()
//                        }

                        runtimes.push_back(current_duration_ns);
                    } else {
                        int sleep_duration_ns = (int) (0.66 * (((1.0 / _exec_freq) * 1.0e9) - current_duration_ns));
                        std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_duration_ns));
                    }

                    if ((CurrentTime() - last_log_time_ns) > 1.0e9) {
                        double avg_exec_ms =
                                std::accumulate(runtimes.begin(), runtimes.end(), 0.0) / (1.0e6 * runtimes.size());
                        if (_exec_freq_sem.try_acquire_for(std::chrono::duration<ulong, std::milli>(10))) {
                            _measure_exec_freq = 1.0 / (avg_exec_ms / 1.0e3);
                            _exec_freq_sem.release();
                        }
                        if (_debug_v <= AppLogger::DEBUG) {
                            AppLogger::Logger::Log(
                                    "Average " + _thread_name + " execution ms: " + std::to_string(avg_exec_ms),
                                    AppLogger::INFO);
                            AppLogger::Logger::Log("\t Max: " + std::to_string(
                                                           *std::max_element(runtimes.begin(), runtimes.end()) / 1.0e6) + "ms",
                                                   AppLogger::INFO);
                        }
                        last_log_time_ns = CurrentTime();
                        runtimes.clear();
                    }

                }
            } catch (const std::exception& e){
                AppLogger::Logger::Log(_thread_name + " in the catch handler!", AppLogger::SEVERITY::WARNING);
                AppLogger::Logger::Log(e.what(), AppLogger::SEVERITY::WARNING);
            }
            _stay_alive_sem.acquire();
            _interrupted_sem.acquire();
            if (!_stay_alive || _interrupted) {
                _stay_alive_sem.release();
                _interrupted_sem.release();
                Finish();
                break;
            }

            _stay_alive_sem.release();
            _interrupted_sem.release();

            _stop_sem.acquire();
            _stop = false;
            _stop_sem.release();
            Finish();

            AppLogger::Logger::Log("Stay alive enabled, thread " + _thread_name + " will restart");
        }
    };

};