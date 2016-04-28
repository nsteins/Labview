#pragma once

#include <comdef.h>

/**
 * Template class to facilitate handling a one dimensional \c SAFEARRAY structure used in COM.
 * The handling of a \c SAFEARRAY structure in C or C++ can be a bit annoying.
 * This class provides a more natural handling for C++ programmers.
 * You can create a \c SAFEARRAY from your normal C++ array or extract the data.
 * CDSafeArray and CLSafeArray are "shortcuts" for \c SAFEARRAYs with doubles or longs,
 * so you do not have to remember the different values for the \c VARTYPE argument.
 * See the \ref comexample "COM example client" for an example of the usage of CDSafeArray
 * and CLSafeArray.
 * \addindex SAFEARRAY
 */
template<class T>
class CSafeArray
{
public:
	/**
	 * constructor creating a SAFEARRAY and filling it with \a size elements of type
	 * \a vt from the array \a pData.
	 * \code
	 * double dArray[5];
	 * // ... fill dArray ...
	 * CSafeArray<double> SArray(dArray, 6, VT_R8); // copy content of dArray
	 * SomeFunc(SArray); // function needing a SAFEARRAY(double)** pointer as argument
	 * \endcode
	 */
	CSafeArray(T* pData, int size, VARTYPE vt)
	{
		SAFEARRAYBOUND rgsabound[1];
		rgsabound[0].lLbound = 0;
		rgsabound[0].cElements = size;
		m_pSafeArray = SafeArrayCreate(vt, 1, rgsabound);
		if (m_pSafeArray!=NULL)
		{
			for (long i=0; i<size; i++)
			{
				SafeArrayPutElement(m_pSafeArray, &i, &pData[i]);
			}
			m_Size = size;
		}
		else
		{
			m_Size = 0;
		}
	}

	/**
	 * constructor creating a SAFEARRAY with \a size elements of type \a vt.
	 * \code
	 * CSafeArray<long> SArray(12, VT_I4); // create a SAFEARRAY(long) structure with 12 elements
	 * SomeFunc(SArray); // function filling a SAFEARRAY(long)** pointer with data
	 * long lArray[10];
	 * SArray.CopyData(lArray, 10); // we are only interested in the first 10 elements
	 * long lArray2[20];
	 * SArray.CopyData(lArray2, 20); // we will only get the 12 elements of SArray 
	 * \endcode
	 */
	CSafeArray(int size, VARTYPE vt)
	{
		SAFEARRAYBOUND rgsabound[1];
		rgsabound[0].lLbound = 0;
		rgsabound[0].cElements = size;
		m_pSafeArray = SafeArrayCreate(vt, 1, rgsabound);
		if (m_pSafeArray!=NULL)
		{
			m_Size = size;
		}
		else
		{
			m_Size = 0;
		}
	}

	/**
	 * Destructor freeing the resources used by the internal SAFEARRAY structure.
	 */
	~CSafeArray()
	{
		if (m_pSafeArray)
			SafeArrayDestroy(m_pSafeArray);
	}

	/**
	 * Copy the data in the \c SAFEARRAY to \a pData.
	 * At most, \a size elements are copied. If the number of elements in the \c SAFEARRAY is
	 * less than \a size, only the content of the \c SAFEARRAY is copied
     * the excess elements in \a pData remain unchanged
	 */
	void CopyData(T* pData, int size)
	{
		T* dum;
		HRESULT hRes = SafeArrayAccessData(m_pSafeArray, (void**) &dum);
		if (SUCCEEDED(hRes))
		{
			for (int i=0; i<min(size, m_Size); i++)
			{
				pData[i] = dum[i];
			}
			SafeArrayUnaccessData(m_pSafeArray);
		}
	}

	/**
	 * Get number of elements in the SAFEARRAY structure.
	 */
	int GetSize() { return m_Size; }

	/**
	 * Convert CSafeArray to a SAFEARRAY** pointer
	 */
	operator SAFEARRAY** () { return &m_pSafeArray; }

protected:
	SAFEARRAY* m_pSafeArray;
	int m_Size;
};

/**
 * Class handling a SAFEARRAY of doubles: this is a "shortcut" for CSafeArray<double>
 * provided so you do not have to remember the \c VARTYPE argument values for the the constructor.
 */
class CDSafeArray : public CSafeArray<double>
{
public:
	/**
	 * Constructor creating a CSafeArray<double> object and filling it with \a size elements
	 * from the array \a pData.
	 */
	CDSafeArray(double* pData, int size) : CSafeArray<double>(pData, size, VT_R8) {}
	/**
	 * Constructor creating a CSafeArray<double> object with \a size elements.
	 */
	explicit CDSafeArray(int size) : CSafeArray<double>(size, VT_R8) {}
};

/**
 * Class handling a SAFEARRAY of longs: this is a "shortcut" for CSafeArray<long>
 * provided so you do not have to remember the \c VARTYPE argument values for the the constructor.
 */
class CLSafeArray : public CSafeArray<long>
{
public:
	/**
	 * Constructor creating a CSafeArray<long> object and filling it with \a size elements
	 * from the array \a pData.
	 */
	CLSafeArray(long* pData, int size) : CSafeArray<long>(pData, size, VT_I4) {}
	/**
	 * Constructor creating a CSafeArray<long> object with \a size elements.
	 */
	explicit CLSafeArray(int size) : CSafeArray<long>(size, VT_I4) {}
};
