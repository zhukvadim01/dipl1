#ifndef _gl_math_adv_h
#define _gl_math_adv_h

//#include <cmath>
#include <cmath>


constexpr double SQRT_2				= 1.4142135623730;		//square root of 2
constexpr double SQRT_3				= 1.7320508075688;		//square root of 3
constexpr double GRAVITY_CONST		= 9.807;				//gravitational constant
constexpr double Pi					= 3.1415926535897932384626433832795;


//#ifdef WIN32

//    #include <float.h>

//    //PACKAGE		:GL
//    //FUNCTION		:finite
//    //DESCRIPTION	:Defines whether a floating number is finite
//    //INPUTS		:_x - floating number
//    //RETURN		:1 if _x is finite, 0 otherwise
//    template <class _TYPE>
//    int finite(_TYPE _x)
//    {
//        return _finite(_x);
//    }

//#else

//    //#include <ieeefp.h>

//#endif


////PACKAGE		:GL
////FUNCTION		:abs
////DESCRIPTION	:Calculates absolute value of the given integer number
////INPUTS		:_x - integer number
////RETURN		:Absolute value of _x
//template <class _TYPE>
//_TYPE abs(_TYPE _x)
//{
//    return (_x < 0)	?	-_x
//                    :	_x;
//}


////PACKAGE		:GL
////FUNCTION		:fabs
////DESCRIPTION	:Calculates absolute value of the given floating number
////INPUTS		:_x - floating number
////RETURN		:Absolute value of _x
//template <class _TYPE>
//_TYPE fabs(_TYPE _x)
//{
//    return abs(_x);
//}


//PACKAGE		:GL
//FUNCTION		:sqr
//DESCRIPTION	:Calculates square of a number
//INPUTS		:_x - given number
//RETURN		:Square of _x
template <class _TYPE>
double sqr(_TYPE _x)
{
    return _x * _x;
}


////PACKAGE		:GL
////FUNCTION		:sqrt
////DESCRIPTION	:Calculates square root of a given number
////INPUTS		:_x - given number
////RETURN		:Square root of _x
//template <class _TYPE>
//_TYPE sqrt(_TYPE _x)
//{
//    return _TYPE(sqrt(double(_x)));
//}


//PACKAGE		:GL
//FUNCTION		:cube
//DESCRIPTION	:Calculates cube of a number
//INPUTS		:_x - given number
//RETURN		:Cube of _x
template <class _TYPE>
_TYPE cube(_TYPE _x)
{
    return _x * _x * _x;
}


//PACKAGE		:GL
//FUNCTION		:cubrt
//DESCRIPTION	:Calculates cube root of a given number
//INPUTS		:_x - given number
//RETURN		:Cube root of _x
template <class _TYPE>
_TYPE cubrt(_TYPE _x)
{
    if(_x >= 0)
        return _TYPE(pow(double(_x), 1./3));
    else
        return _TYPE(-pow(double(-_x), 1./3));
}


//PACKAGE		:GL
//FUNCTION		:signum
//DESCRIPTION	:Calculates signum of a given number
//INPUTS		:_x - given number
//RETURN		:Signum of _x
template <class _TYPE>
_TYPE signum(_TYPE _x)
{
    if (_x > 0)
    {
        return _TYPE(1);
    }
    else
    {
        if (_x < 0)
        {
            return _TYPE(-1);
        }
        else
        {
            return _TYPE(0);
        }
    }
}


//PACKAGE		:GL
//FUNCTION		:ValuesAreValid
//DESCRIPTION	:Checks the validity of the values
//INPUTS		:_Values[] - array of values
//				:_Amount - values quantity
//				:_Threshold - propriety threshold. Air tasks usually do not work with numbers greater than 1.2e7. Default 0 means threshold absence
//RETURN		:1 if the values are valid, 0 otherwise
template <class _TYPE>
bool ValuesAreValid(_TYPE _Values[], int _Amount = 1, double _Threshold = 0)
{
	for (int i = 0; i < _Amount; i++)
	{
        if (	std::isfinite(_Values[i]) == 0
			||	(	fabs(_Values[i]) > _Threshold
			&&	_Threshold != 0)	
			)
			return false;
	}

	return true;
}

//PACKAGE		:GL
//FUNCTION		:MaxArrayElement
//DESCRIPTION	:Finds the greatest element in the array
//INPUTS		:_Array[] - given array
//				:_ArraySize - quantity of elements in the array
//				:_pIndexOfMaxEl_ - pointer to the index of the greatest element
//RETURN		:The greatest element of the array
template <class TYPE_VAL, class TYPE_IND>
TYPE_VAL MaxArrayElement(TYPE_VAL _Array[], int _ArraySize, TYPE_IND* _pIndexOfMaxEl_ = 0)
{
	int IndexOfMaxEl = 0;

	TYPE_VAL MaxEl = _Array[IndexOfMaxEl];

	for (int i = 0; i < _ArraySize; i++)
	{
		if (_Array[i] > MaxEl)
		{
			MaxEl = _Array[i];
			IndexOfMaxEl = i;
		}
	}

	if (_pIndexOfMaxEl_ != 0)
		*_pIndexOfMaxEl_ = IndexOfMaxEl;

	return MaxEl;
}

#endif
