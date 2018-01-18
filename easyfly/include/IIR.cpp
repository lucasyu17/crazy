#include "IIR.h"

#define _USE_MATH_DEFINES //PI
 void IIR_I::reset()  
{  
    for(int i = 0; i <= m_num_order; i++)  
    {  
        m_pNum[i] = 0.0;  
    }  
    for(int i = 0; i <= m_den_order; i++)  
    {  
        m_pDen[i] = 0.0;  
    }  
    //calculO2param(sample_freq, cutoff_freq);
}  
IIR_I::IIR_I()  
{  
    m_pNum = nullptr;  
    m_pDen = nullptr;  
    m_px = nullptr;  
    m_py = nullptr;  
    m_num_order = -1;  
    m_den_order = -1; 
    _cutoff_freq = 0.0f;
    _delay_element_1 = 0.0f;       // buffered sample -1
    _delay_element_2 = 0.0f;       // buffered sample -2 
    _a1 = 0.0f;
    _a2 = 0.0f;
    _b0 = 0.0f;
    _b1 = 0.0f;
    _b2 = 0.0f; 
};  
/** \brief 
 * 
 * \param num 分子多项式的系数，升序排列,num[0] 为常数项 
 * \param m 分子多项式的阶数 
 * \param den 分母多项式的系数，升序排列,den[0] 为常数项 
 * \param m 分母多项式的阶数 
 * \return 
 */  
void IIR_I::setPara(float num[], int num_order, float den[], int den_order)  
{  
    delete[] m_pNum;  
    delete[] m_pDen;  
    delete[] m_px;  
    delete[] m_py;  
    m_pNum = new float[num_order + 1];  
    m_pDen = new float[den_order + 1];  
    m_num_order = num_order;  
    m_den_order = den_order;  
    m_px = new float[num_order + 1];  
    m_py = new float[den_order + 1];  
    for(int i = 0; i <= m_num_order; i++)  
    {  
        m_pNum[i] = num[i];  
        m_px[i] = 0.0;  
    }  
    for(int i = 0; i <= m_den_order; i++)  
    {  
        m_pDen[i] = den[i];  
        m_py[i] = 0.0;  
    }  
}  
  
/** \brief 滤波函数，采用直接I型结构 
 * 
 * \param data 传入输入数据 
 * \return 滤波后的结果 
 */  
float IIR_I::filter(float data)  
{  
    m_py[0] = 0.0; // 存放滤波后的结果  
    m_px[0] = data;  
    for(int i = 0; i <= m_num_order; i++)  
    {  
        m_py[0] = m_py[0] + m_pNum[i] * m_px[i];  
    }  
    for(int i = 1; i <= m_den_order; i++)  
    {  
        m_py[0] = m_py[0] - m_pDen[i] * m_py[i];  
    }  
    for(int i = m_num_order; i >= 1; i--)  
    {  
        m_px[i] = m_px[i-1];  
    }  
    for(int i = m_den_order; i >= 1; i--)  
    {  
        m_py[i] = m_py[i-1];  
    }  
    return m_py[0];  
}  
  
float IIR_I::filterO2(float sample)
{
	if (_cutoff_freq <= 0.0f) {
		// no filtering
		return sample;
	}

	// do the filtering
	float delay_element_0 = sample - _delay_element_1 * _a1 - _delay_element_2 * _a2;

	/*if (!PX4_ISFINITE(delay_element_0)) {
		// don't allow bad values to propagate via the filter
		delay_element_0 = sample;
	}*/

	float output = delay_element_0 * _b0 + _delay_element_1 * _b1 + _delay_element_2 * _b2;

	_delay_element_2 = _delay_element_1;
	_delay_element_1 = delay_element_0;

	// return the value.  Should be no need to check limits
	return output;
}

  
void IIR_I::resp(float data_in[], int M, float data_out[], int N)  
  {  
      int i, k, il;  
      for(k = 0; k < N; k++)  
      {  
          data_out[k] = 0.0;  
          for(i = 0; i <= m_num_order; i++)  
          {  
              if( k - i >= 0)  
              {  
                  il = ((k - i) < M) ? (k - i) : (M - 1);  
                  data_out[k] = data_out[k] + m_pNum[i] * data_in[il];  
              }  
          }  
          for(i = 1; i <= m_den_order; i++)  
          {  
              if( k - i >= 0)  
              {  
                  data_out[k] = data_out[k] - m_pDen[i] * data_out[k - i];  
              }  
          }  
      }  
  }  
  void IIR_I::calculO2param(float sample_freq, float cutoff_freq) //order 2 filter
  {
  	
  	_cutoff_freq = cutoff_freq;

	if (_cutoff_freq <= 0.0f) {
	}
	else{

	float fr = sample_freq / _cutoff_freq;
	float ohm = tan(M_PI / fr);
	float c = 1.0f + 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm;
	_b0 = ohm * ohm / c;
	_b1 = 2.0f * _b0;
	_b2 = _b0;
	_a1 = 2.0f * (ohm * ohm - 1.0f) / c;
	_a2 = (1.0f - 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm) / c;
	}
  }