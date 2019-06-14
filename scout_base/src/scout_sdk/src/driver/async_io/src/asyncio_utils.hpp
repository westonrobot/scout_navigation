/**
 * @brief MAVConn async serial utility class
 * @file async_utils.hpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * libmavconn
 * Copyright 2013,2014,2015,2016 Vladimir Ermakov, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#ifndef ASYNCIO_UTILS_HPP
#define ASYNCIO_UTILS_HPP

#include <string>
#include <algorithm>
#include <iostream>

namespace wescore
{
template <typename... Args>
std::string format(const std::string &fmt, Args... args)
{
    // C++11 specify that string store elements continously
    std::string ret;

    auto sz = std::snprintf(nullptr, 0, fmt.c_str(), args...);
    ret.reserve(sz + 1);
    ret.resize(sz); // to be sure there have room for \0
    std::snprintf(&ret.front(), ret.capacity() + 1, fmt.c_str(), args...);
    return ret;
}

template <typename... Args>
bool set_this_thread_name(const std::string &name, Args &&... args)
{
    auto new_name = format(name, std::forward<Args>(args)...);

#ifdef __APPLE__
    return pthread_setname_np(new_name.c_str()) == 0;
#else
    pthread_t pth = pthread_self();
    return pthread_setname_np(pth, new_name.c_str()) == 0;
#endif
}

/**
 * Parse host:port pairs
 */
void url_parse_host(std::string host,
                    std::string &host_out, int &port_out,
                    const std::string def_host, const int def_port)
{
    std::string port;

    auto sep_it = std::find(host.begin(), host.end(), ':');
    if (sep_it == host.end())
    {
        // host
        if (!host.empty())
        {
            host_out = host;
            port_out = def_port;
        }
        else
        {
            host_out = def_host;
            port_out = def_port;
        }
        return;
    }

    if (sep_it == host.begin())
    {
        // :port
        host_out = def_host;
    }
    else
    {
        // host:port
        host_out.assign(host.begin(), sep_it);
    }

    port.assign(sep_it + 1, host.end());
    port_out = std::stoi(port);
}

/**
 * Parse ?ids=sid,cid
 */
void url_parse_query(std::string query)
{
    const std::string ids_end("ids=");
    std::string sys, comp;

    if (query.empty())
        return;

    auto ids_it = std::search(query.begin(), query.end(),
                              ids_end.begin(), ids_end.end());
    if (ids_it == query.end())
    {
        std::cerr << "URL: unknown query arguments" << std::endl;
        return;
    }

    std::advance(ids_it, ids_end.length());
    auto comma_it = std::find(ids_it, query.end(), ',');
    if (comma_it == query.end())
    {
        std::cerr << "URL: no comma in ids= query" << std::endl;
        return;
    }

    sys.assign(ids_it, comma_it);
    comp.assign(comma_it + 1, query.end());
}
} // namespace wescore

#endif /* ASYNCIO_UTILS_HPP */
