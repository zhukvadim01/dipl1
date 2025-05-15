//PACKAGE		:GL.
//FILE			:GLArray.h
//AUTHOR		:Radchenko Igor, 2001.
//DESCRIPTION	:Header file for CGLArray and CGLArrayFix class.

#ifndef _GLArray_h_
#define _GLArray_h_

#include "common/CODefine.h"

#include <cassert>
#include <cstddef>
#include <cstdlib>

//PACKAGE		:GL.
//CLASS			:CGLArray.
//DESCRIPTION	:Dinamic array with check index range.
template <class T>
class CGLArray
{
public:

//PACKAGE		:GL.
//FUNCTION		:CGLArray( void );
//DESCRIPTION	:Constructor.
//INPUTS			:None.
//RETURNS			:None.
	CGLArray( void );

//PACKAGE		:GL.
//FUNCTION		: CGLArray( const CGLArray<T>& arr );
//DESCRIPTION	:Constructor of copy.
//INPUTS			:arr - Reference to CGLArray.
//RETURNS			:None.
	CGLArray( const CGLArray<T>& arr	//Reference to CGLArray
		);

//PACKAGE		:GL.
//FUNCTION		: explicit CGLArray( size_t size , const T& v = T() );
//DESCRIPTION	:Constructor.
//INPUTS			:size - Initial array size, v - Reference to array element.
//RETURNS			:None.
	explicit CGLArray( size_t size ,	//Initial array size.
		const T& v = T()	//Reference to array element.
		);

//PACKAGE		:GL.
//FUNCTION		:~CGLArray( void );
//DESCRIPTION	:Destructor.
//INPUTS			:None.
//RETURNS			:None.
	~CGLArray( void );

//PACKAGE		:GL.
//FUNCTION		:size_t	size( void ) const;.
//DESCRIPTION	:Selector of array size.
//INPUTS			:None.
//RETURNS			:Size of array.
	size_t	size( void ) const;

	void operator*()
	{
		assert(0);
	}

//PACKAGE		:GL.
//FUNCTION		:void	resize( size_t n, const T& x = T() );
//DESCRIPTION	:Sets initial size of array.
//INPUTS			:n - Size of an array, Reference to array element.
//RETURNS			:None.
	void	resize( size_t n,	//Size of an array
		const T& x = T()	//Reference to array element.
		);

    void	Reset( );

	T& operator []( int n );
	const T& operator []( int n ) const;

	CGLArray& operator=( const CGLArray<T>& arr )
	{
		try
		{
			if( m_data != 0 )
			{
				if( m_owner == true )
				{
					delete	[] m_data; 
				}
				m_data = 0; m_size = 0; m_owner = true;
			}
			m_data = arr.m_data;
			m_size = arr.m_size;
			m_owner = false;
		}
		catch(...)
		{
			G_ASSERT(0);
		}
		return	*this;
	}

	class iterator;
	friend class iterator;

//PACKAGE		:GL.
//CLASS		:CGLArray<T>::iterator
//DESCRIPTION	:CGLArray iterator.
	class iterator
	{
	public:
//PACKAGE		:GL.
//FUNCTION		:iterator( void )
//DESCRIPTION	:Constructor.
//INPUTS			:None.
//RETURNS			:None.
		iterator( void ) : m_index(-1), m_pParent(0) {}

//PACKAGE		:GL.
//FUNCTION		:iterator( int index, CGLArray* pParent )
//DESCRIPTION	:Constructor.
//INPUTS			:index - iterator index, pParent - pointer to outer CGLArray object.
//RETURNS			:None.
		iterator( int index,	//iterator index.
			CGLArray* pParent	//pointer to outer CGLArray object.
			) :
		m_index(index),
		m_pParent(pParent){}

//PACKAGE		:GL.
//FUNCTION		:iterator( const iterator& it )
//DESCRIPTION	:Constructor of copy.
//INPUTS			:it - Reference to source iterator.
//RETURNS			:None.
		iterator( const iterator& it	//Reference to source iterator.
			) :
		m_index(it.m_index),
		m_pParent(it.m_pParent){}

//PACKAGE		:GL.
//FUNCTION		:void setParent(CGLArray* pParent)
//DESCRIPTION	:Sets pointer to outer CGLArray object.
//INPUTS			:pParent - pointer to outer CGLArray object.
//RETURNS			:None.
		inline
		void setParent( CGLArray* pParent	//pointer to outer CGLArray object.
			){ m_pParent = pParent; }

		T& operator *( void )
		{
			G_ASSERT( m_index >= 0 && (size_t)m_index < m_pParent->size() && m_pParent != 0 );
			return m_pParent->m_data[m_index];
		}
		T* operator ->( void )
		{
			G_ASSERT( m_index >= 0 && (size_t)m_index < m_pParent->size() && m_pParent != 0 );
			return &m_pParent->m_data[m_index];
		}
		iterator& operator ++( void )
		{
			G_ASSERT( m_index >= 0 && (size_t)m_index < m_pParent->size() && m_pParent != 0 );
			m_index++;
			if( m_index >= m_pParent->size() )
			{
				m_index = -1;
			}
			return *this;
		}
		iterator operator ++( int )
		{
			iterator tmp = *this;
			++*this;
			return tmp;
		}
		bool operator ==( const iterator& it ) const
		{
			return ( m_index == it.m_index && m_pParent == it.m_pParent );
		}
		bool operator != ( const iterator& it ) const
		{
			return !(*this == it);
		}
	protected:
		int			m_index;
		CGLArray*	m_pParent;
	};

	iterator begin( void )
	{
		return	(m_size == 0)? iterator(-1, this) : iterator(0, this);	
	}
	iterator end( void )
	{
		return	iterator(-1, this);
	}

private:
	T*		m_data;					//Pointer to array elements.
	size_t	m_size;					//Size of array.
	bool	m_owner;				//Sign if class is pointer owner.
};

/****************    CGLArray implementation   ***********************/
template <class T>
CGLArray<T>::CGLArray( void ) :
     m_data(0), m_size(0), m_owner(true)
{
  // Nothing to do.
}

template <class T>
CGLArray<T>::CGLArray( const CGLArray<T>& arr ) :
	m_owner(true)
{
	try
	{
		m_data = new	T[arr.m_size];
		if( m_data != 0 )
		{
			m_size = arr.m_size;
		}
		else
		{
			G_ASSERT(0);
			return;
		}
		for(size_t i=0;i<m_size;i++)
		{
			m_data[i] = arr.m_data[i];
		}
	}
	catch(...)
	{
		G_ASSERT(0);
	}
}

template <class T>
CGLArray<T>::CGLArray( size_t size , const T& v) :
    m_data(0),m_size(0),  m_owner(true)
{
	try
	{
		m_data = new	T[size];
		if( m_data != 0 )
		{
			m_size = size;
		}
		else
		{
			G_ASSERT(0);
			return;
		}
		for(size_t i=0;i<m_size;i++)
		{
			m_data[i] = (T&)v;
		}
	}
	catch(...)
	{
		G_ASSERT(0);
	}
}

template <class T>
CGLArray<T>::~CGLArray( void )
{
	try
	{
		if( m_data != 0 )
		{
			if( m_owner == true )
			{
				delete	[] m_data; 
			}
		}
		m_data = 0; m_size = 0; m_owner = true;
	}
	catch(...)
	{
		G_ASSERT(0);
	}
}

template <class T>
void CGLArray<T>::resize( size_t n, const T& x )
{
	try
	{
		if( m_size == 0 && m_data == 0 )
		{
			m_data = new	T[n];
			if( m_data != 0 )
			{
				m_size = n;
			}
			else
			{
				G_ASSERT(0);
				return;
			}
			for(size_t i=0;i<m_size;i++)
			{
				m_data[i] = x;
			}
		}
		else
		{
			G_ASSERT(0);
		}
	}
	catch(...)
	{
		G_ASSERT(0);
	}
}

template <class T>
void	CGLArray<T>::Reset( )
{
    try
    {
        for(size_t i=0;i<m_size;i++)
        {
            m_data[i] = T();
        }
    }
    catch(...)
    {
        G_ASSERT(0);
    }
}

template <class T> inline
size_t CGLArray<T>::size( void ) const
{
	return	m_size;
}

template <class T> inline
T& CGLArray<T>::operator []( int n )
{
	G_ASSERT(m_data != 0 && n >= 0 && (size_t)n < m_size);
	return m_data[n];
}

template <class T> inline
const T& CGLArray<T>::operator []( int n ) const
{
	G_ASSERT(m_data != 0 && n >= 0 && (size_t)n < m_size);
	return m_data[n];
}


/*	Example:
	struct	STest
	{
	};

	STest*	arr;			<- equal ->		CGLArray<STest>	arr;
	arr = new STest[5];						arr.resize(5);
*/

//PACKAGE		:GL.
//CLASS			:CGLArrayFix.
//DESCRIPTION	:Static array with check index range.
template <class T, size_t countCell>
class CGLArrayFix
{
public:

//PACKAGE		:GL.
//FUNCTION		:CGLArrayFix( void );
//DESCRIPTION	:Constructor.
//INPUTS			:None.
//RETURNS			:None.
	CGLArrayFix( void );

//PACKAGE		:GL.
//FUNCTION		:CGLArrayFix( const CGLArrayFix<T, countCell>& arr );
//DESCRIPTION	:Constructor of copy.
//INPUTS			:arr - Reference to source CGLArrayFix.
//RETURNS			:None.
	CGLArrayFix( const CGLArrayFix<T, countCell>& arr );

	void operator*()
	{
		assert(0);
	}

//PACKAGE		:GL.
//FUNCTION		:size_t	size( void ) const;
//DESCRIPTION	:Selector of CGLArrayFix size.
//INPUTS			:None.
//RETURNS			:CGLArrayFix size.
	size_t	size( void ) const;

//PACKAGE		: GL
//FUNCTION		: Reset();
//DESCRIPTION	: Reset all values to zero, size to countCell
//INPUTS		: None
//RETURNS		: None
    void Reset();

	T& operator []( int n );
	const T& operator []( int n ) const;

    CGLArrayFix<T, countCell>& operator=( const CGLArrayFix<T, countCell>& arr )
	{
        G_ASSERT(countCell == arr.size());
		try
		{
			for(size_t i=0; i<m_size; i++)
			{
				m_data[i] = arr.m_data[i];
			}
		}
		catch(...)
		{
			G_ASSERT(0);
		}
		return	*this;
	}

	class iterator;
	friend class iterator;

//PACKAGE		:GL.
//CLASS		:CGLArrayFix<T>::iterator
//DESCRIPTION	:CGLArray iterator.
	class iterator
	{
	public:

//PACKAGE		:GL.
//FUNCTION		:iterator( void )
//DESCRIPTION	:Constructor.
//INPUTS			:None.
//RETURNS			:None.
		iterator( void ) : m_index(-1), m_pParent(0) {}

//PACKAGE		:GL.
//FUNCTION		:iterator( int index, CGLArrayFix* pParent )
//DESCRIPTION	:Constructor.
//INPUTS			:index - iterator index, pParent - pointer to outer CGLArray object.
//RETURNS			:None.
		iterator( int index,	//iterator index
			CGLArrayFix* pParent	//pointer to outer CGLArrayFix object.
			) :
		m_index(index),
		m_pParent(pParent){}

//PACKAGE		:GL.
//FUNCTION		:iterator( const iterator& it )
//DESCRIPTION	:Constructor of copy.
//INPUTS			:index - iterator index, pParent - pointer to outer CGLArray object.
//RETURNS			:None.
		iterator( const iterator& it	//
			) :
		m_index(it.m_index),
		m_pParent(it.m_pParent){}

//PACKAGE		:GL.
//FUNCTION		:void setParent( CGLArrayFix* pParent );
//DESCRIPTION	:Sets pointer to outer CGLArrayFix object.
//INPUTS			:pParent - pointer to outer CGLArrayFix object.
//RETURNS			:None.
		inline
		void setParent( CGLArrayFix* pParent	//pointer to outer CGLArrayFix object.
			){ m_pParent = pParent; }

		T& operator *( void )
		{
			G_ASSERT( m_index >= 0 && (size_t)m_index < m_pParent->size() && m_pParent != 0 );
			return m_pParent->m_data[m_index];
		}
		T* operator ->( void )
		{
			G_ASSERT( m_index >= 0 && (size_t)m_index < m_pParent->size() && m_pParent != 0 );
			return &m_pParent->m_data[m_index];
		}
		iterator& operator ++( void )
		{
			G_ASSERT( m_index >= 0 && (size_t)m_index < m_pParent->size() && m_pParent != 0 );
			m_index++;
			if( m_index >= m_pParent->size() )
			{
				m_index = -1;
			}
			return *this;
		}
		iterator operator ++( int )
		{
			iterator tmp = *this;
			++*this;
			return tmp;
		}
		bool operator ==( const iterator& it ) const
		{
			return ( m_index == it.m_index && m_pParent == it.m_pParent );
		}
		bool operator != ( const iterator& it ) const
		{
			return !(*this == it);
		}
	protected:
		int			m_index;
		CGLArrayFix*	m_pParent;
	};

	iterator begin( void )
	{
		return	( m_size == 0 )? iterator(-1, this) : iterator(0, this);	
	}
	iterator end( void )
	{
		return	iterator(-1, this);
	}

private:
	T		m_data[countCell];		//Array of elements.		
	size_t	m_size;					//Size of array.
};

//*************   CGLArrayFix implementation  ***********************
template <class T, size_t countCell>
CGLArrayFix<T,countCell>::CGLArrayFix( void ) :
	m_size(countCell)
{
	try
	{
		G_ASSERT( countCell > 0 );
		for(size_t i=0;i<m_size;i++)
		{
			m_data[i] = T();
		}
	}
	catch(...)
	{
		G_ASSERT(0);
	}
}

template <class T, size_t countCell>
CGLArrayFix<T,countCell>::CGLArrayFix( const CGLArrayFix<T, countCell>& arr ):m_size(arr.size())
{
	try
	{
		G_ASSERT( countCell > 0 );
        for(size_t i=0;i<m_size;i++)
		{
			m_data[i] = arr.m_data[i];
		}
	}
	catch(...)
    {
		G_ASSERT(0);
	}
}

template <class T, size_t countCell> inline
size_t CGLArrayFix<T,countCell>::size( void ) const
{
	return	m_size;
}

template <class T, size_t countCell> inline
void CGLArrayFix<T, countCell>::Reset()
{
    try
    {
        G_ASSERT( countCell > 0 );
        m_size = static_cast<size_t>(countCell);
        for(size_t i=0;i<m_size;i++)
        {
            m_data[i] = T();
        }
    }
    catch(...)
    {
        G_ASSERT(0);
    }
}

template <class T, size_t countCell> inline
T& CGLArrayFix<T,countCell>::operator []( int n )
{
	G_ASSERT(n >= 0 && (size_t)n < m_size);
	return m_data[n];
}

template <class T, size_t countCell> inline
const T& CGLArrayFix<T,countCell>::operator []( int n ) const
{
	G_ASSERT(n >= 0 && (size_t)n < m_size);
	return m_data[n];
}


/*	Example:
	struct	STest
	{
		STest(){}
	};

	STest	arr[5];			<- equal -> CGLArrayFix<STest, 5>	arr;	

	STest	arr[5][10];		<- equal -> CGLArrayFix<CGLArrayFix<STest, 10>, 5>	arr;	

*/

//****************	crossing table for conversion target number.  *************
//			conversion DPS target number to SC4I target number.
extern CGLArrayFix<unsigned short, 1000>	g_FCC_to_SC4I;
//extern	unsigned short	g_FCC_to_MCC[256];
//			conversion SC4I target number to DPS target number.
extern CGLArrayFix<unsigned char, 1000>		g_SC4I_to_FCC;
//extern	unsigned char	g_MCC_to_FCC[1000];
//*****************************************************************************

template <class T>
class CGLVector : public std::vector<T>
{
public:
    CGLVector() : std::vector<T>() {}
    CGLVector(size_t count) : std::vector<T>(count) {}

    CGLVector( size_t count, const T& value) : std::vector<T>(count, value) {}

    void Reset() {
        std::vector<T>::assign(std::vector<T>::size(), T());
    }
    void Reset(const T& value) {
        std::vector<T>::assign(std::vector<T>::size(), value);
    }
};

#endif //_GLArray_h_
