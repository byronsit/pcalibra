//
// Created by bohuan on 19-7-3.
//
#pragma once

#ifndef PCALIBRA_PARAMETER_H
#define PCALIBRA_PARAMETER_H

/**
 * @brief 一大堆相关的参数
 * @todo 要改为从proto或什么里面读取这些数值*/

namespace pcalibra{
static double ARG0 = 10; ///< 反射率强度最低要求。达不到则滤除的点。 在杆子检测的时候用到
static double ARG1 = 0.7; ///< 2D搜索半径，这个举例内的点，会被归到一起.
static double ARG2 = 8; ///< 拟合直线最小需要的点的数量，记住是雷达静止放着，所以点应该多一些

}//namespace

#endif //PCALIBRA_PARAMETER_H
