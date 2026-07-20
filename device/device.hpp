/**
 * @file device.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-05-18
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#ifndef DEVICE_HPP
#define DEVICE_HPP

#include <cstdint>

namespace device {

class Device {};

template<uint32_t ID, Device* device>
struct Device_Dispatcher_Entry {
    static constexpr uint32_t id = ID;
    static constexpr auto callback = Callback;
};

/* ******************************** 设备调度器 ******************************** */

// 声明调度器
template<typename... Entry_List>
struct Dispatcher;

// 递归基准
template<>
struct Dispatcher<> {
    static void Dispatch(uint32_t id, const uint8_t* buf) {}
};

// 递归模板展开
template<typename Entry, typename... Entry_List>
struct Dispatcher<Entry, Entry_List...> {
    static void Dispatch(uint32_t id, const uint8_t* buf) {
        if (id == Entry::id) {
            Entry::callback(id, buf);
        } else {
            Dispatcher<Entry_List...>::Dispatch(id, buf);
        }
    }
};

} // namespace device

#endif
