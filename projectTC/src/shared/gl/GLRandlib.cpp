#include "GLRandlib.h"

double GLRandLib::Rand()
{
    return std::uniform_real_distribution<>{0, RAND_MAX}(gen);
}

double GLRandLib::Rand(double Begin, double End)
{
    return std::uniform_real_distribution<>{Begin, End}(gen);
}

double GLRandLib::Normal(double Mo, double Disp)
{
    auto result = std::normal_distribution<>{Mo, Disp}(gen);
    Save(result);
    return result;
}

double GLRandLib::Expont(double Theta)
{
    auto result = std::exponential_distribution<>{Theta}(gen);
    Save(result);
    return result;
}

double GLRandLib::Rel(double Sigma)
{
    double x;
    do
      x=Rand();//Rand(0,1);
    while(x==0);

    double dRetValue = sqrt(-2*Sigma*Sigma*log(x));

    Save( dRetValue );

    return dRetValue;
}

int GLRandLib::Poissn(double Lamda)
{
    auto result = std::poisson_distribution<>{Lamda}(gen);
    Save(result);
    return result;
}
