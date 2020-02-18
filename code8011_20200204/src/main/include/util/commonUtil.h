#pragma once

/**
 * 简单的数据信号过滤，例如操控手柄的信息过滤
 * @param axisVal  摇柄信号的过滤
 * 小于0.1且大于-0.1的信号，屏蔽掉。为缓和0-0.1之间的突变，
 * 先减去0.1，再乘以1.11，使信号更加平稳
 */ 

 double DB(double axisVal);
/**
 * 超过峰值直接过滤掉
 */ 
 double Cap(double value, double peak);