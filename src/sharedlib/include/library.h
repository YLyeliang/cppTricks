//
// Created by 叶亮 on 2022/5/19.
//

#ifndef SHAREDLIB_LIBRARY_H
#define SHAREDLIB_LIBRARY_H

// 打印 Hello World!
void hello();

// 使用可变模版参数求和
template<typename T>
T sum(T t) {
    return t;
}
template<typename T, typename... Types>
T sum(T first, Types... rest) {
    return first + sum<T>(rest...);
}

void ProducerConsumerDemo();

void IODemo();

#endif
