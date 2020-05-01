/*****************************************************************************
********************************** DMath.h ***********************************
*****************************************************************************/

#if !defined(__DMATH_H__)
#define __DMATH_H__

/*****************************************************************************
******************************  I N C L U D E  *******************************
*****************************************************************************/

#include <cmath>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

//#include "DConfig.h"

const double dSqrt2 = sqrt(2.0);

template <typename T>
T round(T Num)
   {
   return ((Num > 0.0) ? floor(Num + 0.5) : ceil(Num - 0.5));
   }
      
/*****************************************************************************
***************************** Angle Conversions ******************************
*****************************************************************************/

const double dPI = 3.14159265358979323846;
const float fPI = 3.141593f;
const double dRadToDeg = 180.0 / dPI;
const float fRadToDeg = 180.0f / fPI;
const double dDegToRad = dPI / 180.0;
const float fDegToRad = fPI / 180.0f;

inline double RadToDeg(double dRad)
   {
   return (dRad * dRadToDeg);
   }

inline double DegToRad(double dDeg)
   {
   return (dDeg * dDegToRad);
   }

inline float RadToDeg(float fRad)
   {
   return (fRad * fRadToDeg);
   }

inline float DegToRad(float fDeg)
   {
   return (fDeg * fDegToRad);
   }

/*****************************************************************************
****************************** class DThreshold ******************************
*****************************************************************************/

/*
   Abstract base class for doing simple comparisons.  Encapsulates the
   threhshold value to which comparisons are made.  The type T must implement
   the needed operator for that type.  Since the Apply function is virtual,
   collections of differing threshold operations may be handled through
   pointers.
*/

template <typename T>
class DThreshold
   {
   public :
      static const unsigned int m_nVersion = 0;
      
      DThreshold() = default;
      
      explicit DThreshold(T Threshold) : m_Threshold(Threshold)
         {
         return;
         }
      
      DThreshold(const DThreshold& src) : m_Threshold(src.m_Threshold)
         {
         return;
         }
      
      virtual ~DThreshold() = default;
      
      DThreshold& operator=(const DThreshold& rhs)
         {
         if (this != &rhs)
            {
            m_Threshold = rhs.m_Threshold;
            } // end if

         return (*this);
         }

      bool operator==(const DThreshold& rhs) const
         {
         return (m_Threshold == rhs.m_Threshold);
         }

      bool operator<(const DThreshold& rhs) const
         {
         return (m_Threshold < rhs.m_Threshold);
         }

      bool operator>(const DThreshold& rhs) const
         {
         return (m_Threshold > rhs.m_Threshold);
         }
        
      bool operator<=(const DThreshold& rhs) const
         {
         return (m_Threshold <= rhs.m_Threshold);
         }
         
      bool operator>=(const DThreshold& rhs) const
         {
         return (m_Threshold >= rhs.m_Threshold);
         }
         
      T GetThreshold() const
         {
         return (m_Threshold);
         }
      
      void SetThreshold(T Threshold)
         {
         m_Threshold = Threshold;
         }
      
      virtual bool Apply(T Test) const = 0;

      template<class Archive>
      void serialize(Archive& ar, const unsigned int /* nVersion */)
         {
         ar & boost::serialization::make_nvp("Threshold", m_Threshold);
         return;   
         }

   protected :
      T m_Threshold;
      
   private :
   };  // End of class DThreshold

//BOOST_SERIALIZATION_ASSUME_ABSTRACT(DThreshold<T>);

/* Handle the version information */

namespace boost {
namespace serialization {
#if 0
template <typename T>
struct is_wrapper<DThreshold<T>> : mpl::false_
   {
   };
#endif
template<typename T>
struct version<DThreshold<T>>
   {
      typedef mpl::int_<DThreshold<T>::m_nVersion> type;
      typedef mpl::integral_c_tag tag;
      BOOST_STATIC_CONSTANT(unsigned int, value = version::type::value);
   };

template<typename T>
const unsigned int version<DThreshold<T>>::value;

} // end namespace serialization
} // end namespace boost

/*****************************************************************************
***************************** class DThresholdLT *****************************
*****************************************************************************/

template <typename T>
class DThresholdLT : public DThreshold<T>
   {
   public :
      static const unsigned int m_nVersion = 0;

      DThresholdLT() = default;

      explicit DThresholdLT(T Threshold) : DThreshold<T>(Threshold)
         {
         return;
         }
      
       DThresholdLT(const DThresholdLT& src) : DThreshold<T>(src)
         {
         return;
         }
 
      virtual ~DThresholdLT() = default;
      
      DThresholdLT& operator=(const DThresholdLT& rhs)
         {
         DThreshold<T>::operator=(rhs);

         return (*this);
         }
       
      virtual bool Apply(T Test) const
         {
         return (Test < this->m_Threshold);
         }

      template<class Archive>
      void serialize(Archive& ar, const unsigned int /* nVersion */)
         {
         ar & boost::serialization::make_nvp( "DThreshold",
               boost::serialization::base_object<DThreshold<T>>(*this));
         return;
         }
      
   protected :
   
   private :
   };  // End of class DThresholdLT

/* Handle the version information */

namespace boost {
namespace serialization {

template<typename T>
struct version<DThresholdLT<T>>
   {
      typedef mpl::int_<DThresholdLT<T>::m_nVersion> type;
      typedef mpl::integral_c_tag tag;
      BOOST_STATIC_CONSTANT(unsigned int, value = version::type::value);
   };

template<typename T>
const unsigned int version<DThresholdLT<T>>::value;

} // end namespace serialization
} // end namespace boost

/*****************************************************************************
***************************** class DThresholdLE *****************************
*****************************************************************************/

template <typename T>
class DThresholdLE : public DThreshold<T>
   {
   public :
      static const unsigned int m_nVersion = 0;

      DThresholdLE() = default;
 
      explicit DThresholdLE(T Threshold) : DThreshold<T>(Threshold)
         {
         return;
         }
      
       DThresholdLE(const DThresholdLE& src) : DThreshold<T>(src)
         {
         return;
         }
 
      virtual ~DThresholdLE() = default;
      
      DThresholdLE& operator=(const DThresholdLE& rhs)
         {
         DThreshold<T>::operator=(rhs);

         return (*this);
         }
       
      virtual bool Apply(T Test) const
         {
         return (Test <= this->m_Threshold);
         }

      template<class Archive>
      void serialize(Archive& ar, const unsigned int /* nVersion */)
         {
         ar & boost::serialization::make_nvp( "DThreshold",
               boost::serialization::base_object<DThreshold<T>>(*this));
         return;
         }
      
   protected :
   
   private :
   };  // End of class DThresholdLE

/* Handle the version information */

namespace boost {
namespace serialization {

template <typename T>
struct is_wrapper<DThresholdLE<T>> : mpl::false_
   {
   };

template<typename T>
struct version<DThresholdLE<T>>
   {
   typedef mpl::int_<DThresholdLE<T>::m_nVersion> type;
   typedef mpl::integral_c_tag tag;
   BOOST_STATIC_CONSTANT(unsigned int, value = version::type::value);
   };

template<typename T>
const unsigned int version<DThresholdLE<T>>::value;

} // end namespace serialization
} // end namespace boost

/*****************************************************************************
***************************** class DThresholdGE *****************************
*****************************************************************************/

template <typename T>
class DThresholdGE : public DThreshold<T>
   {
   public :
      static const unsigned int m_nVersion = 0;

      DThresholdGE() = default;
 
      explicit DThresholdGE(T Threshold) : DThreshold<T>(Threshold)
         {
         return;
         }
      
       DThresholdGE(const DThresholdGE& src) : DThreshold<T>(src)
         {
         return;
         }
 
      virtual ~DThresholdGE() = default;
      
      DThresholdGE& operator=(const DThresholdGE& rhs)
         {
         if (this != &rhs)
            {
            DThreshold<T>::operator=(rhs);
            } // end if

         return (*this);
         }
       
      virtual bool Apply(T Test) const
         {
         return (Test >= this->m_Threshold);
         }

      template<class Archive>
      void serialize(Archive& ar, const unsigned int /* nVersion */)
         {
         ar & boost::serialization::make_nvp( "DThreshold",
                  boost::serialization::base_object<DThreshold<T>>(*this));
         return;
         }
      
   protected :
   
   private :
   };  // End of class DThresholdGE

/* Handle the version information */

namespace boost {
namespace serialization {

template <typename T>
struct is_wrapper<DThresholdGE<T>> : mpl::false_
   {
   };

template<typename T>
struct version<DThresholdGE<T>>
   {
   typedef mpl::int_<DThresholdGE<T>::m_nVersion> type;
   typedef mpl::integral_c_tag tag;
   BOOST_STATIC_CONSTANT(unsigned int, value = version::type::value);
   };

template<typename T>
const unsigned int version<DThresholdGE<T>>::value;

} // end namespace serialization
} // end namespace boost

/*****************************************************************************
***************************** class DThresholdGT *****************************
*****************************************************************************/

template <typename T>
class DThresholdGT : public DThreshold<T>
   {
   public :
      static const unsigned int m_nVersion = 0;

      DThresholdGT() = default;
 
      explicit DThresholdGT(T Threshold) : DThreshold<T>(Threshold)
         {
         return;
         }
      
       DThresholdGT(const DThresholdGT& src) : DThreshold<T>(src)
         {
         return;
         }
 
      virtual ~DThresholdGT() = default;
      
      DThresholdGT& operator=(const DThresholdGT& rhs)
         {
         if (this != &rhs)
            {
            DThreshold<T>::operator=(rhs);
            } // end if

         return (*this);
         }
       
      virtual bool Apply(T Test) const
         {
         return (Test > this->m_Threshold);
         }

      template<class Archive>
      void serialize(Archive& ar, const unsigned int /* nVersion */)
         {
         ar & boost::serialization::make_nvp( "DThreshold",
               boost::serialization::base_object<DThreshold<T>>(*this));
         return;
         }
      
   protected :
   
   private :
   };  // End of class DThresholdGT

/* Handle the version information */

namespace boost {
namespace serialization {

template<typename T>
struct version<DThresholdGT<T>>
   {
   typedef mpl::int_<DThresholdGT<T>::m_nVersion> type;
   typedef mpl::integral_c_tag tag;
   BOOST_STATIC_CONSTANT(unsigned int, value = version::type::value);
   };

template<typename T>
const unsigned int version<DThresholdGT<T>>::value;

} // end namespace serialization
} // end namespace boost

/*****************************************************************************
****************************** class DRangeTest ******************************
*****************************************************************************/

/*
   Class to provide checking if a value for a type is between the lower and
   upper limits.  If lower is greater than upper, the range becomes "circular"
   or "not".
*/

template <typename T>
class DRangeTest
   {
   public :
      static const unsigned int m_nVersion = 0;

      DRangeTest() : m_bCircular(false)
         {
         return;
         }
 
      DRangeTest(T Lower, T Upper, bool bCircular = false)
            : m_Lower(Lower), m_Upper(Upper), m_bCircular(bCircular)
         {
         return;
         }
      
      virtual ~DRangeTest() = default;

      DRangeTest(const DRangeTest& src) : m_Lower(src.m_Lower),
            m_Upper(src.m_Upper), m_bCircular(src.m_bCircular)
         {
         return;
         }
            
      DRangeTest& operator=(const DRangeTest& rhs)
         {
         if (this != &rhs)
            {
            m_Lower = rhs.m_Lower;
            m_Upper = rhs.m_Upper;
            m_bCircular = rhs.m_bCircular;
            } // end if

         return (*this);
         }
      
      void SetRange(T Lower, T Upper)
         {
         m_Lower.SetThreshold(Lower);
         m_Upper.SetThreshold(Upper);

         return;
         }
       
      void SetLower(T Lower)
         {
         m_Lower.SetThreshold(Lower);

         return;
         }

      void SetUpper(T Upper)
         {
         m_Upper.SetThreshold(Upper);

         return;
         }

      T GetLower() const
         {
         return (m_Lower.GetThreshold());
         }

      T GetUpper() const
         {
         return (m_Upper.GetThreshold());
         }
       
      bool IsCircular() const
         {
         return (m_bCircular);
         }
      
      void SetCircular(bool bCircular)
         {
         m_bCircular = bCircular;

         return;
         }
      
      virtual bool Apply(T Test) const
         {
         bool bRet = false;
         if (m_bCircular && (m_Lower > m_Upper))
            {
            bRet = (m_Lower.Apply(Test) || m_Upper.Apply(Test));
            } // end if
         else
            {
            bRet = (m_Lower.Apply(Test) && m_Upper.Apply(Test));
            } // end else

         return (bRet);
         }

      template<class Archive>
      void serialize(Archive& ar, const unsigned int /* nVersion */)
         {
         ar & boost::serialization::make_nvp("Lower", m_Lower);
         ar & boost::serialization::make_nvp("Upper", m_Upper);
         ar & boost::serialization::make_nvp("Circular", m_bCircular);

         return;   
         }

   protected :
        DThresholdGE<T> m_Lower;
        DThresholdLE<T> m_Upper;
        bool m_bCircular;
        
   private :
   };  // End of class DRangeTest

/* Handle the version information */

namespace boost {
namespace serialization {

template<typename T>
struct version<DRangeTest<T>>
   {
   typedef mpl::int_<DRangeTest<T>::m_nVersion> type;
   typedef mpl::integral_c_tag tag;
   BOOST_STATIC_CONSTANT(unsigned int, value = version::type::value);
   };

template<typename T>
const unsigned int version<DRangeTest<T>>::value;

} // end namespace serialization
} // end namespace boost


#if 0                    
/*****************************************************************************
************************** class DCircularRangeTest **************************
*****************************************************************************/

template <typename T>
class DCircularRangeTest : public DRangeTest<T>
   {
   public :
      DCircularRangeTest(T Lower = 0, T Upper = 0, T Min = 0, T Max = 0)
            : DRangeTest(Lower, Upper), m_Min(Min), m_Max(Max)
         {
         return;
         }

      DCircularRangeTest(const DCircularRangeTest& src)
            : DRangeTest(src)
         {
         m_Min = src.m_Min;
         m_Max = src.m_Max;
         return;
         }
            
      ~DCircularRangeTest()
         {
         return;
         }

      DCircularRangeTest& operator=(const DCircularRangeTest& rhs)
         {
         DRangeTest::operator=(rhs);
         m_Min = rhs.m_Min;
         m_Max = rhs.m_Max;
         return;
         }
      
      void SetMinMax(T Min, T Max)
        {
        m_Min = Min;
        m_Max = Max;
        return;
        }
      
      void SetMin(T Min)
        {
        m_Min = Min;
        return;
        }

      void SetMax(T Max)
        {
        m_Max = Max;
        return;
        }

      T GetMin() const
         {
         return (m_Min);
         }

      T GetMax() const
         {
         return (m_Max);
         }
       
      virtual bool Apply(T Test) const
         {
         return (m_Lower.Apply(Test) && m_Upper.Apply(Test));
         }
       
   protected :
      T m_Min;
      T m_Max;
   
   private :
   };  // End of class DCircularRangeTest
#endif

                                                                                                          
#endif // __DMATH_H__
