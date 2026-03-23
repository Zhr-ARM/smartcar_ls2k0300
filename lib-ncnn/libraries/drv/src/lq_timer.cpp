#include "lq_timer.hpp"
#include "lq_assert.hpp"

/********************************************************************************
 * @brief   定时器无参构造函数.
 * @param   none.
 * @return  none.
 * @example lq_timer timer;
 * @note    none.
 ********************************************************************************/
lq_timer::lq_timer() : is_running_(false), target_ns_(0), callback_(nullptr), timer_thread_(&lq_timer::timer_handler_thread, this)
{
}

/********************************************************************************
 * @brief   定时器析构函数.
 * @param   none.
 * @return  none.
 * @example none.
 * @note    变量生命周期结束, 函数自动执行.
 ********************************************************************************/
lq_timer::~lq_timer()
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->thread_running_ = false;  // 标记线程退出
    this->is_running_ = false;      // 标记定时器停止

    this->cv_.notify_all();         // 唤醒等待的线程
    if (this->timer_thread_.joinable())
    {
        this->timer_thread_.join(); // 等待线程退出
    }
}

/********************************************************************************
 * @brief   设置秒级定时器.
 * @param   _s  : 秒数.
 * @param   _cb : 回调函数.
 * @return  none.
 * @example lq_timer timer;
 *          timer.set_seconds_s(1, callback_function);
 * @note    none.
 ********************************************************************************/
bool lq_timer::set_seconds_s(uint64_t _s, const timer_callback& _cb)
{
    if (_s == 0)
    {
        lq_log_error("Error: Seconds must be greater than 0!");
        return false;
    }
    return this->timer_update(_s * 1000000000ULL, _cb);
}

/********************************************************************************
 * @brief   毫秒级定时器.
 * @param   _ms : 毫秒数.
 * @param   _cb : 回调函数.
 * @return  none.
 * @example lq_timer timer;
 *          timer.set_seconds_ms(1000, callback_function);
 * @note    none.
 ********************************************************************************/
bool lq_timer::set_seconds_ms(uint64_t _ms, const timer_callback& _cb)
{
    if (_ms == 0)
    {
        lq_log_error("Error: Milliseconds must be greater than 0!");
        return false;
    }
    return this->timer_update(_ms * 1000000ULL, _cb);
}

/********************************************************************************
 * @brief   停止定时器.
 * @param   none.
 * @return  none.
 * @example lq_timer timer;
 *          timer.stop();
 * @note    none.
 ********************************************************************************/
bool lq_timer::stop()
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->is_running_ = false;
    this->target_ns_ = 0;
    this->callback_ = nullptr;
    this->cv_.notify_all(); // 唤醒线程进入休眠
    return true;
}

/********************************************************************************
 * @brief   获取定时器状态.
 * @param   none.
 * @return  none.
 * @example lq_timer timer;
 *          bool status = timer.is_running();
 * @note    none.
 ********************************************************************************/
bool lq_timer::is_running() const
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->is_running_;
}

/********************************************************************************
 * @brief   更新定时器配置.
 * @param   _ns : 定时器间隔.
 * @param   _cb : 回调函数.
 * @return  bool.
 * @example lq_timer timer;
 *          bool status = timer.timer_update(5, callback_function);
 * @note    none.
 ********************************************************************************/
bool lq_timer::timer_update(uint64_t _ns, const timer_callback& _cb)
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    // 先停止原有定时
    this->is_running_ = false;
    this->cv_.notify_all();     // 唤醒线程读取新配置

    // 更新参数
    this->target_ns_ = _ns;
    this->callback_  = _cb;
    // 有效回调则启动定时, 否则保持休眠
    if (this->callback_)
    {
        this->is_running_ = true;
        this->cv_.notify_all(); // 唤醒线程
        lq_log_info("Precision timer updated (ns: %llu), started.", this->target_ns_);
    } else {
        lq_log_info("Precision timer updated (ns: %llu), no callback, sleeping.", this->target_ns_);
    }
    return true;
}

/********************************************************************************
 * @brief   定时器线程.
 * @param   none.
 * @return  none.
 * @example none.
 * @note    内部函数.
 ********************************************************************************/
void lq_timer::timer_handler_thread()
{
    lq_log_info("Precision timer thread started (ID: %lu)", std::this_thread::get_id());

    while (true)
    {
        // 加锁
        std::unique_lock<std::mutex> lock(this->mutex_);

        // 无有效定时/无回调: 进入休眠
        if (!this->thread_running_ || !this->is_running_ || !this->callback_)
        {
            if (!this->thread_running_) break;
            this->cv_.wait(lock);   // 休眠直到被唤醒
            continue;
        }

        // 读取配置并解锁
        const uint64_t interval_ns    = this->target_ns_;
        const timer_callback callback = this->callback_;    // 拷贝回调, 避免锁持有过久
        lock.unlock();

        // 开始时间获取
        auto loop_start = std::chrono::steady_clock::now();

        // 执行回调函数
        try {
            this->callback_();
        } catch (...) {
            lq_log_error("Callback execption!");
        }

        // 回调结束计算耗时
        auto loop_end = std::chrono::steady_clock::now();
        uint64_t used_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(loop_end - loop_start).count();

        // 超时则警告, 每超则补时长
        if (used_ns > interval_ns)
        {
            lq_log_warn("Timeout! Used: %llu ns, Target: %llu ns", used_ns, interval_ns);
        } else {
            this->timer_sleep_ns(interval_ns - used_ns);
        }
    }
    lq_log_info("Timer thread exited.");
}

/********************************************************************************
 * @brief   休眠.
 * @param   _ns : 休眠时间.
 * @return  none.
 * @example lq_timer timer;
 *          timer.timer_sleep_ns(1000000000);
 * @note    内部函数.
 ********************************************************************************/
void lq_timer::timer_sleep_ns(uint64_t _ns)
{
    if (_ns == 0) return;
    // 剩余时间 > 100us, 用 clock_nanosleep(内核级精准休眠)
    const uint64_t threshold_ns = 100 * 1000;   // 100微秒阈值
    if (_ns > threshold_ns)
    {
        struct timespec req;
        req.tv_sec  = _ns / 1000000000ULL;
        req.tv_nsec = _ns % 1000000000ULL;
        clock_nanosleep(CLOCK_MONOTONIC, 0, &req, nullptr);
        _ns -= (req.tv_sec * 1000000000ULL + req.tv_nsec);
    }
    // 剩余时间 <= 100us, 忙等补精度
    if (_ns > 0)
    {
        auto end = std::chrono::steady_clock::now() + std::chrono::nanoseconds(_ns);
        while (std::chrono::steady_clock::now() < end)
        {
            std::this_thread::yield();
        }
    }
}
