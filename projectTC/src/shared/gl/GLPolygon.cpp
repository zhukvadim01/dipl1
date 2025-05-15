//#include "GLPch.h"
#include "GLPolygon.h"
#include "GLGeometry.h"
#include "GLMath.h"
#include <limits.h>

static
inline
GLPointDouble2D operator -( const GLPointDouble2D& a, const GLPointDouble2D& b )
{
    return GLPointDouble2D(a.x - b.x, a.y - b.y);
}

/*********************************************************************

                   GLPolygon implementation

**********************************************************************/

GLPolygon::GLPolygon( qint32 size ) : std::vector<GLPointDouble2D>( size )
{
    // Nothing to do.
}

GLPolygon::GLPolygon() : std::vector<GLPointDouble2D>( 0 )
{
    // Nothing to do.
}


GLPolygon::~GLPolygon()
{
    // Nothing to do.
}

qint32 GLPolygon::getSize()
{
    return size();
}

bool GLPolygon::ptInPolygon( long x, long y ) const
{
    bool answer = false;
    quint32 i;
    qint32 next_i;
    GLPointDouble2D	pt_vect,side_vect;
    double side_cotangens;

    for (i = 0; i < capacity(); i++)
    {
        if (i == size())
          break;
        next_i = (i+1)%size();
        if( /* РўРѕС‡РєР° СЂР°СЃРїРѕР»РѕР¶РµРЅР° РїРѕ РІРµСЂС‚РёРєР°Р»Рё РјРµР¶РґСѓ РєРѕРЅС†Р°РјРё
                    СЃС‚РѕСЂРѕРЅС‹ РїРѕР»РёРіРѕРЅР° ? */
                ((y > at(i).y)==(y <= at(next_i).y)) )
        {
            // Р’РµРєС‚РѕСЂ, РЅР°РїСЂР°РІР»РµРЅРЅС‹Р№ РѕС‚ i-Р№ С‚РѕС‡РєРё Рє pt.
            pt_vect = GLPointDouble2D(x,y) - at(i);

            // Р’РµРєС‚РѕСЂ, РЅР°РїСЂР°РІР»РµРЅРЅС‹Р№ РѕС‚ i-Р№ С‚РѕС‡РєРё Рє i+1-Р№
            side_vect = at(next_i) - at(i);
            // РљРѕС‚Р°РЅРіРµРЅСЃ СѓРіР»Р° РЅР°РєР»РѕРЅР° СЃС‚РѕСЂРѕРЅС‹ РїРѕР»РёРіРѕРЅР°
            if (side_vect.y)
            {
                side_cotangens = (double)side_vect.x/(double)side_vect.y;
            }
            else
            {
                side_cotangens = LONG_MAX;
            }

            /*
            РњС‹СЃР»РµРЅРЅРѕ СЂР°Р·РґРµР»РёРІ РѕР±Рµ С‡Р°СЃС‚Рё РЅРµСЂР°РІРµРЅСЃС‚РІР° РЅР° pt_vect.y РІРёРґРёРј, С‡С‚Рѕ
            СЃСЂР°РІРЅРёРІР°СЋС‚СЃСЏ РєРѕС‚Р°РЅРіРµРЅСЃС‹, С‚.Рµ. СѓРіР»С‹ РјРµР¶РґСѓ РѕСЃСЊСЋ X Рё РІРµРєС‚РѕСЂР°РјРё
            pt_vect Рё side_vect. РќСѓР¶РЅРѕ РёРјРµС‚СЊ РІ РІРёРґСѓ, С‡С‚Рѕ РїСЂРё РІРѕР·СЂР°СЃС‚Р°РЅРёРё
            СѓРіР»Р° РєРѕС‚Р°РЅРіРµРЅСЃ СѓР±С‹РІР°РµС‚.
            */
            if (pt_vect.x < pt_vect.y*side_cotangens)
            {
                /*
                Р§С‚РѕР±С‹ РїРѕР»СѓС‡РёС‚СЊ СѓС‚РІРµСЂРґРёС‚РµР»СЊРЅС‹Р№ РѕС‚РІРµС‚, РЅРµРѕР±С…РѕРґРёРјРѕ
                Р·Р°Р№С‚Рё СЃСЋРґР° РЅРµС‡РµС‚РЅРѕРµ РєРѕР»РёС‡РµСЃС‚РІРѕ СЂР°Р·.
                */
                answer = !answer;
            }
        }
    }

    return answer;
}

bool GLPolygon::ptBeforePolygon( double x, double y, qint16 course/*in second*/, qint32& DistanceToZone ) const
{
    quint32 i;
    qint32 next_i;
    //qint32 resolve = 0;
    qint16 b1=0,b2=0; // VV_1028, VV_1029:Corrected
    double tempA = 0;  // VV_1067:Corrected
    bool flag_point = false;
    bool flag_distance = false;
    GLPointDouble2D	b1_vect,b2_vect,res_pnt;
    DistanceToZone = 0;

    for ( i = 0; i < capacity(); i++ )
    {
        next_i = (i+1)%size();

        b1_vect = at(i) - GLPointDouble2D(x,y);
        b2_vect = at(next_i) - GLPointDouble2D(x,y);

        if (b1_vect.x != 0  || b1_vect.y != 0)
        {
            b1 = count_Azim(b1_vect);
        }
        if (b2_vect.x != 0  || b2_vect.y != 0)
        {
            b2 = count_Azim(b2_vect);
        }

        //AMCC_fprintf(pZones,"b1 = %d      b2 = %d     course =%d    (I=%d   I+1=%d)\n", b1,b2,course,i,next_i);


        flag_point = false;

        if (CourseInSector(course,b1,b2))
        {
            if ((at(i).y - at(next_i).y) != 0)
            {
                tempA = (at(next_i).x - at(i).x)/(at(next_i).y - at(i).y);
            }

            if ((course == 270 * 60) || (course == 90 * 60))
            {
                if ((at(i).y - at(next_i).y) != 0)
                {
                    res_pnt.y = y;
                    res_pnt.x = y * tempA + at(i).x - at(i).y * tempA;
                    flag_point = true;
                }
                else if (y == at(i).y)
                {
                    res_pnt.y = y;
                    if (fabs(x - at(i).x) > fabs(x - at(next_i).x))
                    {
                        res_pnt.x = at(i).x;
                    }
                    else
                    {
                        res_pnt.x = at(next_i).x;
                    }
                    flag_point = true;
                }
            }
            else if ((course == 180 * 60) || (course == 0))
            {
                if ((at(i).y - at(next_i).y) != 0)
                {
                    //res_pnt.x = x;
                    if (fabs(x-at(i).x) < fabs(x-at(next_i).x))
                    {
                        res_pnt.x = at(i).x;
                    }
                    else
                    {
                        res_pnt.x = at(next_i).x;
                    }
                    res_pnt.y = (x + at(i).y * tempA - at(i).x ) / tempA;
                    flag_point = true;
                }
                else
                {
                    //res_pnt.x = x;
                    if (fabs(x-at(i).x) < fabs(x-at(next_i).x))
                    {
                        res_pnt.x = at(i).x;
                    }
                    else
                    {
                        res_pnt.x = at(next_i).x;
                    }
                    res_pnt.y = at(i).y;
                    flag_point = true;
                }
            }
            else if (tan(course/ 60./ 180. * PI) == tempA)
            {
                if (GLdistance(GLPointDouble2D(x,y),at(i)) > GLdistance(GLPointDouble2D(x,y),at(next_i)))
                {
                    res_pnt = at(i);
                }
                else
                {
                    res_pnt = at(next_i);
                }
            }
            else
            {
                if ((at(i).y - at(next_i).y) != 0)
                {
                    //res_pnt.y = (at(i).x - at(i).y * tempA - x + y *tan(course/ 60. / 180. * PI))/(tan(course/ 60. / 180. * PI) - tempA);
                    //res_pnt.x = tan(course/ 60./ 180. * PI) * res_pnt.y + x - y;
                    res_pnt.y = ( (at(i).x - at(i).y * tempA - x) * tan(course/ 60. / 180. * PI) + y ) / (1 - tempA * tan(course/ 60. / 180. * PI)); // y = (tg(alpf)*(x0-tempA*y0-x) + y)/(1-tg(alpf)*tempA)
                    res_pnt.x = res_pnt.y * tempA + at(i).x - at(i).y * tempA;
                    flag_point = true;
                }
                else
                {
                    res_pnt.y = at(i).y;
                    res_pnt.x = res_pnt.y * tempA + at(i).x - at(i).y * tempA;
                    flag_point = true;
                }
            }

            //AMCC_fprintf(pZones,"Course in this sector ");
            if (flag_point)
            {
                //AMCC_fprintf(pZones,"and point of peresechenia (%d:%d)\n ",(int)res_pnt.x,(int)res_pnt.y);
                //AMCC_fprintf(pZones,"check point of peresechenia \n ");
                if ((std::min(at(i).x,at(next_i).x) > res_pnt.x) || ( res_pnt.x > std::max(at(i).x,at(next_i).x)))
                {
                    if (fabs(at(i).x - res_pnt.x) < fabs(at(next_i).x - res_pnt.x))
                    {
                        res_pnt.x = at(i).x;
                    }
                    else
                    {
                        res_pnt.x = at(next_i).x;
                        //AMCC_fprintf(pZones,"new(x) point of peresechenia (%d:%d)\n ",(int)res_pnt.x,(int)res_pnt.y);
                    }
                }
                if ((std::min(at(i).y,at(next_i).y) > res_pnt.y) || ( res_pnt.y > std::max(at(i).y,at(next_i).y)))
                {
                    if (fabs(at(i).y - res_pnt.y) < fabs(at(next_i).y - res_pnt.y))
                    {
                        res_pnt.y = at(i).y;
                    }
                    else
                    {
                        res_pnt.y = at(next_i).y;
                    //AMCC_fprintf(pZones,"new(y) point of peresechenia (%d:%d)\n ",(int)res_pnt.x,(int)res_pnt.y);
                    }
                }
                if ((GLdistance(res_pnt,GLPointDouble2D(x,y)) < DistanceToZone) || (!flag_distance))
                {
                    //AMCC_fprintf(pZones,"Prev minDist=%d   ",DistanceToZone);
                    DistanceToZone = GLdistance(res_pnt,GLPointDouble2D(x,y));
                    //AMCC_fprintf(pZones,"Now minDist=%d   \n",DistanceToZone);
                    flag_distance = true;
                }
            }
            else
            {
                //AMCC_fprintf(pZones,", but we cant calculate point peresechenia\n ");
            }
        }
    }

    if (flag_distance)
    {
        return true;
    }
    else
    {
        return false;
    }
}