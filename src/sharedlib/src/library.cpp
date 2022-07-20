#include "library.h"
#include "condition_variable"
#include "future"
#include "mutex"
#include "queue"
#include "thread"
#include "unistd.h"
#include <iostream>
#include <shared_mutex>
using namespace std;
/***
 * 多线程实现
 ***/

// shared_mutex读写锁把对共享资源的访问者划分成读者和写者，多个读线程能同时读取共享资源，但只有一个写线程能同时读写共享资源
// shared_mutex通过lock_shared，unlock_shared进行读者的锁定与解锁；通过lock，unlock进行写者的锁定与解锁。

// 读写锁
// shared_mutex s_m;
// std::string book;
// void read()
// {
//     s_m.lock_shared();
//     cout << book;
//     s_m.unlock_shared();
// }
// void write()
// {
//     s_m.lock();
//     book = "new context";
//     s_m.unlock();
// }


// 互斥锁实例
// NOTE:不推荐实直接去调用成员函数lock(),使用lock_guard或者unique_lock则能避免忘记解锁
mutex m;// 实例化m对象
void proc1(int a) {
    m.lock();
    cout << "proc1函数正在改写a" << endl;
    cout << "原始a为" << a << endl;
    cout << "现在a为" << a + 2 << endl;
    m.unlock();
}

void proc2(int a) {
    m.lock();
    cout << "proc2函数正在改写a" << endl;
    cout << "原始a为" << a << endl;
    cout << "现在a为" << a + 1 << endl;
    m.unlock();
}

// lock_guard
// 在其构造函数中进行加锁，在其析构函数中进行解锁
void proc1_guard(int a) {
    lock_guard<mutex> g1(m);//用此语句替换了m.lock()；lock_guard传入一个参数时，该参数为互斥量，此时调用了lock_guard的构造函数，申请锁定m
    // 等价于如下：
    // lock_guard可以传入两个参数，使用adopt_lock标识时，需要手动锁定
    // m.lock();
    // lock_guard<mutex> g1(m,adopt_lock);
    cout << "proc1函数正在改写a" << endl;
    cout << "原始a为" << a << endl;
    cout << "现在a为" << a + 2 << endl;
}//此时不需要写m.unlock(),g1出了作用域被释放，自动调用析构函数，于是m被解锁

void proc2_guard(int a) {
    {
        lock_guard<mutex> g2(m);
        cout << "proc2函数正在改写a" << endl;
        cout << "原始a为" << a << endl;
        cout << "现在a为" << a + 1 << endl;
    }//通过使用{}来调整作用域范围，可使得m在合适的地方被解锁
    cout << "作用域外的内容3" << endl;
    cout << "作用域外的内容4" << endl;
    cout << "作用域外的内容5" << endl;
}

void MutexPractice() {
    int a = 0;
    thread t1(proc1, a);
    thread t2(proc2, a);
    t1.join();
    t2.join();
    thread t3(proc1_guard, a);
    thread t4(proc2_guard, a);
}

// unique_lock

// std::unique_lock类似于lock_guard,只是std::unique_lock用法更加丰富，同时支持std::lock_guard()的原有功能。
// 使用std::lock_guard后不能手动lock()与手动unlock();
// 使用std::unique_lock后可以手动lock()与手动unlock();
// std::unique_lock的第二个参数，除了可以是adopt_lock,还可以是try_to_lock与defer_lock;

void proc1_unique(int a) {
    unique_lock<mutex> g1(m, defer_lock);//始化了一个没有加锁的mutex
    cout << "xxxxxxxx" << endl;
    g1.lock();//手动加锁，注意，不是m.lock();注意，不是m.lock(),m已经被g1接管了;
    cout << "proc1函数正在改写a" << endl;
    cout << "原始a为" << a << endl;
    cout << "现在a为" << a + 2 << endl;
    g1.unlock();//临时解锁
    cout << "xxxxx" << endl;
    g1.lock();
    cout << "xxxxxx" << endl;
}//自动解锁

void proc2_unique(int a) {
    unique_lock<mutex> g2(m, try_to_lock);//尝试加锁一次，但如果没有锁定成功，会立即返回，不会阻塞在那里，且不会再次尝试锁操作。
    if (g2.owns_lock()) {                 //锁成功
        cout << "proc2函数正在改写a" << endl;
        cout << "原始a为" << a << endl;
        cout << "现在a为" << a + 1 << endl;
    } else {//锁失败则执行这段语句
        cout << "" << endl;
    }
}//自动解锁

// unique_lock 所有权转移
// {
//     unique_lock<mutex> g2(m,defer_lock);
//     unique_lock<mutex> g3(move(g2));//所有权转移，此时由g3来管理互斥量m
//     g3.lock();
//     g3.unlock();
//     g3.lock();
// }

// condition_variable
// 它的作用是用来同步线程，它的用法相当于编程中常见的flag标志
// 类比到std::condition_variable，A、B两个人约定notify_one为行动号角，A就等着（调用wait(),阻塞）,只要B一调用notify_one，A就开始行动（不再阻塞）。
// wait(locker) : wait函数需要传入一个std::mutex
// wait函数会自动调用 locker.unlock() 释放锁（因为需要释放锁，所以要传入mutex）并阻塞当前线程
//
// cond.notify_one(): 随机唤醒一个等待的线程
//
// cond.notify_all(): 唤醒所有等待的线程

// 异步线程
// std::async是一个函数模板，用来启动一个异步任务，它返回一个std::future类模板对象，future对象起到了占位作用
// 在调用std::future对象的get()成员函数时，主线程会被阻塞直到异步线程执行结束

double t1(const double a, const double b) {
    double c = a + b;
    sleep(3000);//假设t1函数是个复杂的计算过程，需要消耗3秒
    return c;
}

void AsyncDemo() {
    double a = 23.1;
    double b = 33.1;
    future<double> fu = async(t1, a, b);//创建异步线程线程，并将线程的执行结果用fu占位；
    cout << "正在进行计算" << endl;
    cout << "计算结果马上就准备好，请您耐心等待" << endl;
    cout << "计算结果：" << fu.get() << endl;//阻塞主线程，直至异步线程return
                                             //cout << "计算结果：" << fu.get() << endl;//取消该语句注释后运行会报错，因为future对象的get()方法只能调用一次。
}

// shared_future
// std::future与std::shard_future的用途都是为了占位
// std::future的get()成员函数是转移数据所有权;std::shared_future的get()成员函数是复制数据
// future对象的get()只能调用一次；无法实现多个线程等待同一个异步线程
//  std::shared_future对象的get()可以调用多次；可以实现多个线程等待同一个异步线程


// 原子类型atomic<>
// 原子操作指“不可分割的操作”，也就是说这种操作状态要么是完成的，要么是没完成的，不存在“操作完成了一半”这种状况
// std::atomic<>用来定义一个自动加锁解锁的共享"变量"

//原子类型的简单使用
// std::atomic<bool> b(true);
// b=false;
// store是原子写操作，load是原子读操作。exchange是于两个数值进行交换的原子操作
// 即使使用了std::atomic<>，也要注意执行的操作是否支持原子性


// normal
// 线程创建实例
void proc(int &a) {
    cout << "我是子线程,传入参数为" << a << endl;
    cout << "子线程中显示子线程id为" << this_thread::get_id() << endl;
}

void MfPractice() {
    // 1. create thread
    // std::thread th1(proc);

    // 当线程启动后，需要在相关联的std::thread对象销毁前调用join() or detach()方法
    // 区别是是否等待子线程执行结束
    cout << "我是主线程" << endl;
    int a = 9;
    thread th2(proc, ref(a));//第一个参数为函数名，第二个参数为该函数的第一个参数，如果该函数接收多个参数就依次写在后面。此时线程开始执行。
    cout << "主线程中显示子线程id为" << th2.get_id() << endl;
    //此处执行其他不依赖线程的任务
    th2.join();//此时主线程被阻塞直至子线程执行结束。
    // 使用joinable()来判断join()可否调用
}

// 生产者-消费者
//缓冲区存储的数据类型
struct CacheData {
    //商品id
    int id;
    //商品属性
    string data;
};

queue<CacheData> Q;
//缓冲区最大空间
const int MAX_CACHEDATA_LENGTH = 10;
//互斥量，生产者之间，消费者之间，生产者和消费者之间，同时都只能一个线程访问缓冲区
// mutex m;
condition_variable condConsumer;
condition_variable condProducer;
//全局商品id
int ID = 1;

//消费者动作
void ConsumerActor() {
    unique_lock<mutex> lockerConsumer(m);
    cout << "[" << this_thread::get_id() << "] 获取了锁" << endl;
    while (Q.empty()) {
        cout << "因为队列为空，所以消费者Sleep" << endl;
        cout << "[" << this_thread::get_id() << "] 不再持有锁" << endl;
        //队列空， 消费者停止，等待生产者唤醒
        condConsumer.wait(lockerConsumer);
        cout << "[" << this_thread::get_id() << "] Weak, 重新获取了锁" << endl;
    }
    cout << "[" << this_thread::get_id() << "] ";
    CacheData temp = Q.front();
    cout << "- ID:" << temp.id << " Data:" << temp.data << endl;
    Q.pop();
    condProducer.notify_one();
    cout << "[" << this_thread::get_id() << "] 释放了锁" << endl;
}

//生产者动作
void ProducerActor() {
    unique_lock<mutex> lockerProducer(m);
    cout << "[" << this_thread::get_id() << "] 获取了锁" << endl;
    while (Q.size() > MAX_CACHEDATA_LENGTH) {
        cout << "因为队列为满，所以生产者Sleep" << endl;
        cout << "[" << this_thread::get_id() << "] 不再持有锁" << endl;
        //队列满，生产者停止，等待消费者唤醒
        condProducer.wait(lockerProducer);
        cout << "[" << this_thread::get_id() << "] Weak, 重新获取了锁" << endl;
    }
    cout << "[" << this_thread::get_id() << "] ";
    CacheData temp;
    temp.id = ID++;
    temp.data = "********";
    cout << "+ ID:" << temp.id << " Data:" << temp.data << endl;
    Q.push(temp);
    condConsumer.notify_one();
    cout << "[" << this_thread::get_id() << "] 释放了锁" << endl;
}
//消费者
void ConsumerTask() {
    while (1) {
        ConsumerActor();
    }
}

//生产者
void ProducerTask() {
    while (1) {
        ProducerActor();
    }
}

//管理线程的函数
void Dispatch(int ConsumerNum, int ProducerNum) {
    vector<thread> t_c;
    for (int i = 0; i < ConsumerNum; ++i) {
        t_c.push_back(thread(ConsumerTask));
    }

    vector<thread> t_p;
    for (int i = 0; i < ProducerNum; ++i) {
        t_p.push_back(thread(ProducerTask));
    }

    for (int i = 0; i < ConsumerNum; ++i) {
        if (t_c[i].joinable()) {
            t_c[i].join();
        }
    }

    for (int j = 0; j < ProducerNum; ++j) {
        if (t_p[j].joinable()) {
            t_p[j].join();
        }
    }
}


void ProducerConsumerDemo(){
    //一个消费者线程，5个生产者线程，则生产者经常要等待消费者
    Dispatch(1,5);
}