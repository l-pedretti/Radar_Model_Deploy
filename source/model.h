/*
* DEEPCRAFT Studio 5.2.2102+2abb7d94b69921b66d282c11929c83b8adc99268
* Copyright © 2023- Imagimob AB, All Rights Reserved.
* 
* Generated at 12/13/2024 13:32:48 UTC. Any changes will be lost.
* 
* Model ID  d59ddc00-78f9-4dfb-bce0-2e4b9dd8d9d3
* 
* Memory    Size                      Efficiency
* Buffers   585728 bytes (RAM)        90 %
* State     61648 bytes (RAM)         100 %
* Readonly  4007456 bytes (Flash)     100 %
* 
* Backend              tensorflow
* Keras Version        2.15.0
* Backend Model Type   Sequential
* Backend Model Name   conv2d-medium-balanced-1
* 
* Class Index | Symbol Label
* 0           | unlabelled
* 1           | fall
* 2           | lie
* 
* Layer                          Shape           Type       Function
* Sliding Window (data points)   [15,1024]       float      dequeue
*    window_shape = [15,1024]
*    stride = 1024
*    buffer_multiplier = 1
* Contextual Window (Sliding Window) [15,1024]       float      dequeue
*    contextual_length_sec = 3
*    prediction_freq = 5
* Input Layer                    [15,1024]       float      dequeue
*    shape = [15,1024]
* Reshape                        [15,1024,1]     float      dequeue
*    shape = [15,1024,1]
*    trainable = True
* Convolution 2D                 [8,512,16]      float      dequeue
*    filters = 16
*    kernel_size = [3,3]
*    strides = [2,2]
*    padding = same
*    activation = linear
*    use_bias = False
*    trainable = True
*    weight = float[3,3,1,16]
* Batch Normalization            [8,512,16]      float      dequeue
*    epsilon = 0.001
*    trainable = True
*    scale = True
*    center = True
*    axis = 3
*    gamma = float[16]
*    beta = float[16]
*    mean = float[16]
*    variance = float[16]
* Activation                     [8,512,16]      float      dequeue
*    activation = relu
*    trainable = True
* Convolution 2D                 [8,512,16]      float      dequeue
*    filters = 16
*    kernel_size = [3,3]
*    strides = [1,1]
*    padding = same
*    activation = linear
*    use_bias = False
*    trainable = True
*    weight = float[3,3,16,16]
* Convolution 2D                 [8,512,16]      float      dequeue
*    filters = 16
*    kernel_size = [3,3]
*    strides = [1,1]
*    padding = same
*    activation = linear
*    use_bias = False
*    trainable = True
*    weight = float[3,3,16,16]
* Batch Normalization            [8,512,16]      float      dequeue
*    epsilon = 0.001
*    trainable = True
*    scale = True
*    center = True
*    axis = 3
*    gamma = float[16]
*    beta = float[16]
*    mean = float[16]
*    variance = float[16]
* Activation                     [8,512,16]      float      dequeue
*    activation = relu
*    trainable = True
* Max pooling 2D                 [4,256,16]      float      dequeue
*    pool_size = [2,2]
*    strides = [2,2]
*    padding = valid
*    trainable = True
* Convolution 2D                 [4,256,32]      float      dequeue
*    filters = 32
*    kernel_size = [3,3]
*    strides = [1,1]
*    padding = same
*    activation = linear
*    use_bias = False
*    trainable = True
*    weight = float[3,3,16,32]
* Convolution 2D                 [4,256,32]      float      dequeue
*    filters = 32
*    kernel_size = [3,3]
*    strides = [1,1]
*    padding = same
*    activation = linear
*    use_bias = False
*    trainable = True
*    weight = float[3,3,32,32]
* Batch Normalization            [4,256,32]      float      dequeue
*    epsilon = 0.001
*    trainable = True
*    scale = True
*    center = True
*    axis = 3
*    gamma = float[32]
*    beta = float[32]
*    mean = float[32]
*    variance = float[32]
* Activation                     [4,256,32]      float      dequeue
*    activation = relu
*    trainable = True
* Flatten                        [32768]         float      dequeue
* Dense                          [30]            float      dequeue
*    units = 30
*    use_bias = False
*    activation = linear
*    trainable = True
*    weight = float[32768,30]
* Batch Normalization            [30]            float      dequeue
*    epsilon = 0.001
*    trainable = True
*    scale = True
*    center = True
*    axis = 1
*    gamma = float[30]
*    beta = float[30]
*    mean = float[30]
*    variance = float[30]
* Activation                     [30]            float      dequeue
*    activation = relu
*    trainable = True
* Dropout                        [30]            float      dequeue
*    rate = 0.1
*    trainable = True
* Dense                          [3]             float      dequeue
*    units = 3
*    use_bias = False
*    activation = linear
*    trainable = True
*    weight = float[30,3]
* Activation                     [3]             float      dequeue
*    activation = softmax
*    trainable = True
* 
* Exported functions:
* 
* int IMAI_dequeue(float *restrict data_out)
*    Description: Dequeue features. RET_SUCCESS (0) on success, RET_NODATA (-1) if no data is available, RET_NOMEM (-2) on internal memory error
*    Parameter data_out is Output of size float[3].
* 
* int IMAI_enqueue(const float *restrict data_in)
*    Description: Enqueue features. Returns SUCCESS (0) on success, else RET_NOMEM (-2) when low on memory.
*    Parameter data_in is Input of size float[1024].
* 
* void IMAI_init(void)
*    Description: Initializes buffers to initial state. This function also works as a reset function.
* 
* 
* Disclaimer:
*   The generated code relies on the optimizations done by the C compiler.
*   For example many for-loops of length 1 must be removed by the optimizer.
*   This can only be done if the functions are inlined and simplified.
*   Check disassembly if unsure.
*   tl;dr Compile using gcc with -O3 or -Ofast
*/

#ifndef _IMAI_MODEL_H_
#define _IMAI_MODEL_H_
#ifdef _MSC_VER
#pragma once
#endif

#include <stdint.h>

typedef struct {    
    char *name;
    double TP; // True Positive or Correct Positive Prediction
    double FN; // False Negative or Incorrect Negative Prediction
    double FP; // False Positive or Incorrect Positive Prediction
    double TN; // True Negative or Correct Negative Prediction
    double TPR; // True Positive Rate or Sensitivity, Recall
    double TNR; // True Negative Rate or Specificity, Selectivity
    double PPV; // Positive Predictive Value or Precision
    double NPV; // Negative Predictive Value
    double FNR; // False Negative Rate or Miss Rate
    double FPR; // False Positive Rate or Fall-Out
    double FDR; // False Discovery Rate
    double FOR; // False Omission Rate
    double F1S; // F1 Score
} IMAI_stats;

/*
* Tensorflow Test Set
* 
* (ACC) Accuracy 99.885 %
* (F1S) F1 Score 99.828 %
* 
* Name of class                                            (unlabelled)             fall              lie
* (TP) True Positive or Correct Positive Prediction                   0             1280             1333
* (FN) False Negative or Incorrect Negative Prediction                3                0                0
* (FP) False Positive or Incorrect Positive Prediction                0                1                2
* (TN) True Negative or Correct Negative Prediction                2613             1335             1281
* (TPR) True Positive Rate or Sensitivity, Recall                0.00 %         100.00 %         100.00 %
* (TNR) True Negative Rate or Specificity, Selectivity         100.00 %          99.93 %          99.84 %
* (PPV) Positive Predictive Value or Precision                 100.00 %          99.92 %          99.85 %
* (NPV) Negative Predictive Value                               99.89 %         100.00 %         100.00 %
* (FNR) False Negative Rate or Miss Rate                       100.00 %           0.00 %           0.00 %
* (FPR) False Positive Rate or Fall-Out                          0.00 %           0.07 %           0.16 %
* (FDR) False Discovery Rate                                   100.00 %           0.08 %           0.15 %
* (FOR) False Omission Rate                                      0.11 %           0.00 %           0.00 %
* (F1S) F1 Score                                                 0.00 %          99.96 %          99.93 %
*/


#define IMAI_TEST_AVG_ACC 0.9988532110091743 // Accuracy
#define IMAI_TEST_AVG_F1S 0.9982801776993208 // F1 Score

#define IMAI_TEST_STATS { \
 {name: "(unlabelled)", TP: 0, FN: 3, FP: 0, TN: 2613, TPR: 0, TNR: 1, PPV: 1, NPV: 0.9988532110091, FNR: 1, FPR: 0, FDR: 1, FOR: 0.0011467889908, F1S: 0, }, \
 {name: "fall", TP: 1280, FN: 0, FP: 1, TN: 1335, TPR: 1, TNR: 0.9992514970059, PPV: 0.9992193598750, NPV: 1, FNR: 0, FPR: 0.0007485029940, FDR: 0.0007806401249, FOR: 0, F1S: 0.9996095275283, }, \
 {name: "lie", TP: 1333, FN: 0, FP: 2, TN: 1281, TPR: 1, TNR: 0.9984411535463, PPV: 0.9985018726591, NPV: 1, FNR: 0, FPR: 0.0015588464536, FDR: 0.0014981273408, FOR: 0, F1S: 0.9992503748125, }, \
}

#ifdef IMAI_STATS_ENABLED
static const IMAI_stats IMAI_test_stats[] = IMAI_TEST_STATS;
#endif

/*
* Tensorflow Train Set
* 
* (ACC) Accuracy 100.000 %
* (F1S) F1 Score 100.000 %
* 
* Name of class                                            (unlabelled)             fall              lie
* (TP) True Positive or Correct Positive Prediction                   3             3898             4210
* (FN) False Negative or Incorrect Negative Prediction                0                0                0
* (FP) False Positive or Incorrect Positive Prediction                0                0                0
* (TN) True Negative or Correct Negative Prediction                8108             4213             3901
* (TPR) True Positive Rate or Sensitivity, Recall              100.00 %         100.00 %         100.00 %
* (TNR) True Negative Rate or Specificity, Selectivity         100.00 %         100.00 %         100.00 %
* (PPV) Positive Predictive Value or Precision                 100.00 %         100.00 %         100.00 %
* (NPV) Negative Predictive Value                              100.00 %         100.00 %         100.00 %
* (FNR) False Negative Rate or Miss Rate                         0.00 %           0.00 %           0.00 %
* (FPR) False Positive Rate or Fall-Out                          0.00 %           0.00 %           0.00 %
* (FDR) False Discovery Rate                                     0.00 %           0.00 %           0.00 %
* (FOR) False Omission Rate                                      0.00 %           0.00 %           0.00 %
* (F1S) F1 Score                                               100.00 %         100.00 %         100.00 %
*/


#define IMAI_TRAIN_AVG_ACC 1 // Accuracy
#define IMAI_TRAIN_AVG_F1S 1 // F1 Score

#define IMAI_TRAIN_STATS { \
 {name: "(unlabelled)", TP: 3, FN: 0, FP: 0, TN: 8108, TPR: 1, TNR: 1, PPV: 1, NPV: 1, FNR: 0, FPR: 0, FDR: 0, FOR: 0, F1S: 1, }, \
 {name: "fall", TP: 3898, FN: 0, FP: 0, TN: 4213, TPR: 1, TNR: 1, PPV: 1, NPV: 1, FNR: 0, FPR: 0, FDR: 0, FOR: 0, F1S: 1, }, \
 {name: "lie", TP: 4210, FN: 0, FP: 0, TN: 3901, TPR: 1, TNR: 1, PPV: 1, NPV: 1, FNR: 0, FPR: 0, FDR: 0, FOR: 0, F1S: 1, }, \
}

#ifdef IMAI_STATS_ENABLED
static const IMAI_stats IMAI_train_stats[] = IMAI_TRAIN_STATS;
#endif

/*
* Tensorflow Validation Set
* 
* (ACC) Accuracy 99.968 %
* (F1S) F1 Score 99.952 %
* 
* Name of class                                            (unlabelled)             fall              lie
* (TP) True Positive or Correct Positive Prediction                   0             1533             1561
* (FN) False Negative or Incorrect Negative Prediction                1                0                0
* (FP) False Positive or Incorrect Positive Prediction                0                1                0
* (TN) True Negative or Correct Negative Prediction                3094             1561             1534
* (TPR) True Positive Rate or Sensitivity, Recall                0.00 %         100.00 %         100.00 %
* (TNR) True Negative Rate or Specificity, Selectivity         100.00 %          99.94 %         100.00 %
* (PPV) Positive Predictive Value or Precision                 100.00 %          99.93 %         100.00 %
* (NPV) Negative Predictive Value                               99.97 %         100.00 %         100.00 %
* (FNR) False Negative Rate or Miss Rate                       100.00 %           0.00 %           0.00 %
* (FPR) False Positive Rate or Fall-Out                          0.00 %           0.06 %           0.00 %
* (FDR) False Discovery Rate                                   100.00 %           0.07 %           0.00 %
* (FOR) False Omission Rate                                      0.03 %           0.00 %           0.00 %
* (F1S) F1 Score                                                 0.00 %          99.97 %         100.00 %
*/


#define IMAI_VALIDATION_AVG_ACC 0.9996768982229403 // Accuracy
#define IMAI_VALIDATION_AVG_F1S 0.9995154000083226 // F1 Score

#define IMAI_VALIDATION_STATS { \
 {name: "(unlabelled)", TP: 0, FN: 1, FP: 0, TN: 3094, TPR: 0, TNR: 1, PPV: 1, NPV: 0.9996768982229, FNR: 1, FPR: 0, FDR: 1, FOR: 0.0003231017770, F1S: 0, }, \
 {name: "fall", TP: 1533, FN: 0, FP: 1, TN: 1561, TPR: 1, TNR: 0.9993597951344, PPV: 0.9993481095176, NPV: 1, FNR: 0, FPR: 0.0006402048655, FDR: 0.0006518904823, FOR: 0, F1S: 0.9996739484838, }, \
 {name: "lie", TP: 1561, FN: 0, FP: 0, TN: 1534, TPR: 1, TNR: 1, PPV: 1, NPV: 1, FNR: 0, FPR: 0, FDR: 0, FOR: 0, F1S: 1, }, \
}

#ifdef IMAI_STATS_ENABLED
static const IMAI_stats IMAI_validation_stats[] = IMAI_VALIDATION_STATS;
#endif

#define IMAI_API_QUEUE

// All symbols in order
#define IMAI_SYMBOL_MAP {"unlabelled", "fall", "lie"}

// Model GUID (16 bytes)
#define IMAI_MODEL_ID {0x00, 0xdc, 0x9d, 0xd5, 0xf9, 0x78, 0xfb, 0x4d, 0xbc, 0xe0, 0x2e, 0x4b, 0x9d, 0xd8, 0xd9, 0xd3}

// First nibble is bit encoding, second nibble is number of bytes
#define IMAGINET_TYPES_NONE	(0x0)
#define IMAGINET_TYPES_FLOAT32	(0x14)
#define IMAGINET_TYPES_FLOAT64	(0x18)
#define IMAGINET_TYPES_INT8	(0x21)
#define IMAGINET_TYPES_INT16	(0x22)
#define IMAGINET_TYPES_INT32	(0x24)
#define IMAGINET_TYPES_INT64	(0x28)
#define IMAGINET_TYPES_QDYN8	(0x31)
#define IMAGINET_TYPES_QDYN16	(0x32)
#define IMAGINET_TYPES_QDYN32	(0x34)

// data_in [1024] (4096 bytes)
#define IMAI_DATA_IN_COUNT (1024)
#define IMAI_DATA_IN_TYPE float
#define IMAI_DATA_IN_TYPE_ID IMAGINET_TYPES_FLOAT32
#define IMAI_DATA_IN_SCALE (1)
#define IMAI_DATA_IN_OFFSET (0)
#define IMAI_DATA_IN_IS_QUANTIZED (0)

// data_out [3] (12 bytes)
#define IMAI_DATA_OUT_COUNT (3)
#define IMAI_DATA_OUT_TYPE float
#define IMAI_DATA_OUT_TYPE_ID IMAGINET_TYPES_FLOAT32
#define IMAI_DATA_OUT_SCALE (1)
#define IMAI_DATA_OUT_OFFSET (0)
#define IMAI_DATA_OUT_IS_QUANTIZED (0)

#define IMAI_KEY_MAX (41)



// Return codes
#define IMAI_RET_SUCCESS 0
#define IMAI_RET_NODATA -1
#define IMAI_RET_NOMEM -2

// Exported methods
int IMAI_dequeue(float *restrict data_out);
int IMAI_enqueue(const float *restrict data_in);
void IMAI_init(void);

#endif /* _IMAI_MODEL_H_ */
