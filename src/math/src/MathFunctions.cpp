// created by yel on 2022/5/19
#include "iostream"
#include "shared_mutex"
#include "mutex"
#include "thread"
using namespace std;



double power(double base, int exponent) {
    double result = base;
    int i;
    if (exponent == 0) {
        return 1;
    }
    for (i = 1; i < exponent; ++i) {
        result = result * base;
    }
    return result;
}

void foo(int z) {
    for (int i = 0; i < z; ++i) {
        cout << "使用函数指针作为可调用参数\n";
    }
}

// 可调用对象
class thread_obj {
public:
    void operator()(int x) {
        for (int i = 0; i < x; ++i) {
            cout << "使用函数对象作为可调用参数\n";
        }
    }
};

// 多线程基本知识，多线程调用
void MultiThreadTest() {
    cout << "线程 1、2、3 独立运行" << endl;

    thread th1(foo, 100);

    thread th2(thread_obj(), 100);

    // lambda
    thread th3([](int x) {for (int i =0;i<x;++i)
    cout<<"使用lambda作为可调用参数\n"; }, 100);

    // 调用join会清理线程相关的存储部分，使用joinable()来判断join()可否调用
    cout << "是否可调用线程阻塞 " << th1.joinable() << endl;
    th1.join();

    th2.join();

    th3.join();
}

// shared mutex test
// shared_mutex读写锁把对共享资源的访问者划分成读者和写者，
// 多个读线程能同时读取共享资源，但只有一个写线程能同时读取共享资源

// 通过lock_shared，unlock_shared进行读者的锁定与解锁；
// 通过lock，unlock进行写者的锁定与解锁。
shared_mutex s_m;
string book;
void read(){
    s_m.lock_shared();
    cout<<book;
    s_m.unlock_shared();
}

void write(){
    s_m.lock();
    book = "new context";
    s_m.unlock();
}

// 互斥锁




