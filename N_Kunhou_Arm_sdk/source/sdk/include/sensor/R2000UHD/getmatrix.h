//
// Created by exbot on 9/20/18.
//

#ifndef N_KUNHOU_ARM_GETMATRIX_H
#define N_KUNHOU_ARM_GETMATRIX_H

#include<stdio.h>
#include<math.h>
#include<stdlib.h>
#include<iostream>
#include <vector>

#define debug false

#define MAX 100

void input(int *m,int *n,double *x,double a[2][MAX]); //输入节点个数、所求坐标位置、每个节点的坐标
void makematrix(double a[2][MAX],double c1[MAX][MAX],int n,int m); //生成系数矩阵
void convert(double c1[MAX][MAX],double c2[MAX][MAX],int n,int m); //转置矩阵
void multiply(double c[MAX][MAX],double c2[MAX][MAX],double c1[MAX][MAX],int n,int m,int t); //矩阵乘法(n*m)*(m*t)=(n*t)
void pmatrix(double a[MAX][MAX],int n,int m); //n*m矩阵的输出，作测试用
std::pair<double, double> output(double x[MAX],int n, double des);  //输出多项式，并求出所找的点对应的函数值
void solvematrix(double c[MAX][MAX],double b[MAX][MAX],double x[MAX],int n); //列主元素消去法解线性方程组，c为系数矩阵，b为右端项，x为解向量，n为矩阵次数
//下面函数均为列主元素消去法所用到函数
void solve(double all[MAX][MAX],int n); //用主元素消去法化简矩阵
void swap(double a[MAX],double b[MAX],int n); //交换矩阵的两行元素
void answer(double a[MAX][MAX],double x[MAX],int n);  //输出解向量
#endif //N_KUNHOU_ARM_GETMATRIX_H
