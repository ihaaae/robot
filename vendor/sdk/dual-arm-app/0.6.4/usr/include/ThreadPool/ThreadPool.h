//  [10/10/2017 maxco]
#pragma once
#include <atomic>
#include <condition_variable>
#include <functional>
#include <list>
#include <mutex>
#include <thread>
#include <queue>
#include<future>
#include <type_traits>


namespace Common {
    template<typename... Ts>
    struct MakeVoid {
        typedef void type;
    };
    template<typename... Ts> using VoidType = typename MakeVoid<Ts...>::type;


    class ThreadPool
    {

    public:
        static ThreadPool& getInstance()
        {
            static ThreadPool threadPool{std::thread::hardware_concurrency()};
            return threadPool;
        }
    public:
        ThreadPool(unsigned c)
                : threadCount_(c), threads_(threadCount_), stop_() {
            stop_ = false;
            for (auto &t : threads_)
                t = std::move(std::thread([this] { this->runInThread(); }));
        }

        ~ThreadPool() {
            joinAll();
        }


        void joinAll() {
            {
                std::unique_lock<std::mutex> lock(queueMutex_);
                waitVar_.wait(lock, [this]() -> bool { return queue_.empty(); });
                stop_ = true;
                waitVar_.notify_all();
            }
            for (auto &t : threads_) {
                if (t.joinable())
                    t.join();
            }
        }


        //建议获取值前，查看future对象是否valid，如果valid,那么再get()
        template<class F, class... Args>
        auto addTask(F &&f, Args &&... args)
        -> std::enable_if_t<!std::is_void<typename std::result_of<F(
                Args...)>::type>::value, std::future<typename std::result_of<F(Args...)>::type>> {
            using return_type = typename std::result_of<F(Args...)>::type;
            if (!stop_) {
                std::unique_lock<std::mutex> lock(queueMutex_);
                auto task = std::make_shared<std::packaged_task<return_type()>>(
                        std::bind(std::forward<F>(f), std::forward<Args>(args)...)
						//[&]() {f(std::forward<Args>(args)...); }
                );

                std::future<return_type> res = task->get_future();
                {
                    if (stop_)
                        return std::future<return_type>{};
                    queue_.emplace([task]() { (*task)(); });
                }
                waitVar_.notify_one();
                return res;
            }
            return std::future<return_type>{};
        }

        template<class F, class... Args>
        auto addTask(F &&f, Args &&... args) ->
        std::enable_if_t<std::is_void<typename std::result_of<F(Args...)>::type>::value, VoidType<>> {
            if (!stop_) {
                std::lock_guard<std::mutex> lock(queueMutex_);
                queue_.emplace(std::bind(std::forward<F>(f), std::forward<Args>(args)...));
				//queue_.emplace([&]() {f(std::forward<Args>(args)...); });
                waitVar_.notify_one();
            }
        }


    private:
        unsigned threadCount_;
        std::list<std::thread> threads_;
        std::queue<std::function<void()> > queue_;

        std::atomic_bool stop_;
        std::condition_variable waitVar_;
        std::mutex queueMutex_;


        void runInThread() {
            std::function<void()> run;
            while (true) {
                std::unique_lock<std::mutex> lock(queueMutex_);
                waitVar_.wait(lock, [this]() {
                    return stop_ || !queue_.empty();
                });/*等待线程池结束或者队列非空*/
                if (!queue_.empty()) {
                    run = std::move(queue_.front());
                    queue_.pop();
                    lock.unlock();
                    waitVar_.notify_one();
                    run();
                }
                else if (stop_) {
                    /*如果线程池已结束且队列为空*/
                    lock.unlock();
                    waitVar_.notify_one();
                    return;
                }
            }
        }
    };
}

