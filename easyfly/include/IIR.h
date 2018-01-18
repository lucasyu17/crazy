#include <math.h>
#include <iostream>
class IIR_I  
{ 
private:  
    float *m_pNum;  
    float *m_pDen; 
    float *m_px; 
    float *m_py; 
    int m_num_order;  
    int m_den_order;

    
public: 
    /*float *m_O2a_params;
    float *m_O2b_params;*/
    IIR_I();  
    void reset();  
    void setPara(float num[], int num_order, float den[], int den_order);  
    void resp(float data_in[], int m, float data_out[], int n); 
    float filter(float data); 
    float filterO2(float sample);
    void calculO2param(float sample_freq, float cutoff_freq);

    float  _cutoff_freq;
    float  _delay_element_1;        // buffered sample -1
    float  _delay_element_2;        // buffered sample -2 
    float  _a1;
    float  _a2;
    float  _b0;
    float  _b1;
    float  _b2;
}; 