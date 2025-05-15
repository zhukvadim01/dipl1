#pragma once

#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <signal.h>
#include <stdlib.h>
#include <sys/timeb.h>

class GLRandLib
{
    FILE* pFileID;
    std::mt19937_64 gen{static_cast<std::mt19937_64::result_type>(std::chrono::system_clock::now().time_since_epoch().count())};

public:
    unsigned char Cout;

    GLRandLib(int64_t a_seed = std::chrono::system_clock::now().time_since_epoch().count())
        : pFileID(nullptr)
        , gen{(unsigned long long)a_seed}
    {}

    void Close()
    {
        if (pFileID)
            fclose(pFileID);
    }

    void Save(double dRetValue)
    {
        if (pFileID)
        {
            fprintf(pFileID, "%f\n", dRetValue);
            fflush(pFileID);
        }
    }

    void Save(int lRetValue)
    {
        if (pFileID)
        {
            fprintf(pFileID, "%d\n", lRetValue);
            fflush(pFileID);
        }
    }

    double Rand(void);
    double Rand(double Begin, double End);
    double Expont(double Theta);
    double Normal(double Mo, double Disp);
    int Poissn(double Lamda);
    double Rel(double Sigma);
};
