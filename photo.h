#ifndef _OPENCV3_PHOTO_H_
#define _OPENCV3_PHOTO_H_

#ifdef __cplusplus
#include <opencv2/opencv.hpp>
#include <opencv2/photo.hpp>
extern "C" {
#endif

#include "core.h"

#ifdef __cplusplus

typedef std::vector<cv::Mat> *VectorMat;
typedef std::vector<int> *VectorInt;
typedef std::vector<float> *VectorFloat;

typedef cv::Ptr<cv::MergeMertens> *MergeMertens;
typedef cv::Ptr<cv::MergeDebevec> *MergeDebevec;
typedef cv::Ptr<cv::MergeRobertson> *MergeRobertson;
typedef cv::Ptr<cv::Tonemap> *Tonemap;
typedef cv::Ptr<cv::TonemapDrago> *TonemapDrago;
typedef cv::Ptr<cv::TonemapMantiuk> *TonemapMantiuk;
typedef cv::Ptr<cv::TonemapReinhard> *TonemapReinhard;
typedef cv::Ptr<cv::AlignMTB> *AlignMTB;

#else

typedef void *VectorMat;
typedef void *VectorInt;
typedef void *VectorFloat;

typedef void *MergeMertens;
typedef void *MergeDebevec;
typedef void *MergeRobertson;
typedef void *Tonemap;
typedef void *TonemapDrago;
typedef void *TonemapMantiuk;
typedef void *TonemapReinhard;
typedef void *AlignMTB;
#endif

VectorMat VectorMat_Create();
VectorMat VectorMat_CreateWithCapacity(uint size);
void VectorMat_PushBack(VectorMat m, Mat mat);
void VectorMat_Free(VectorMat m);

VectorInt VectorInt_Create();
VectorInt VectorInt_CreateWithCapacity(uint size);
void VectorInt_PushBack(VectorInt m, int v);
void VectorInt_Free(VectorInt m);

VectorFloat VectorFloat_Create();
VectorFloat VectorFloat_CreateWithCapacity(uint size);
void VectorFloat_PushBack(VectorFloat m, float v);
void VectorFloat_Free(VectorFloat m);


void ColorChange(Mat src, Mat mask, Mat dst, float red_mul, float green_mul, float blue_mul);

void SeamlessClone(Mat src, Mat dst, Mat mask, Point p, Mat blend, int flags);

void IlluminationChange(Mat src, Mat mask, Mat dst, float alpha, float beta);

void TextureFlattening(Mat src, Mat mask, Mat dst, float low_threshold, float high_threshold, int kernel_size);

void FastNlMeansDenoisingColoredMulti(struct Mats src, Mat dst, int imgToDenoiseIndex, int 	temporalWindowSize);
void FastNlMeansDenoisingColoredMultiWithParams(struct Mats src, Mat dst, int imgToDenoiseIndex, int 	temporalWindowSize, float 	h, float 	hColor, int 	templateWindowSize, int 	searchWindowSize );
void FastNlMeansDenoising(Mat src, Mat dst);
void FastNlMeansDenoisingWithParams(Mat src, Mat dst, float h, int templateWindowSize, int searchWindowSize);
void FastNlMeansDenoisingColored(Mat src, Mat dst);
void FastNlMeansDenoisingColoredWithParams(Mat src, Mat dst, float h, float hColor, int templateWindowSize, int searchWindowSize);

MergeMertens MergeMertens_Create();
MergeMertens MergeMertens_CreateWithParams(float contrast_weight, float saturation_weight, float exposure_weight);
void MergeMertens_Process(MergeMertens b, VectorMat src, Mat dst);
void MergeMertens_Close(MergeMertens b);

MergeDebevec MergeDebevec_Create();
void MergeDebevec_Process(MergeDebevec b, VectorMat src, Mat dst, VectorFloat times);
void MergeDebevec_Close(MergeDebevec b);

MergeRobertson MergeRobertson_Create();
void MergeRobertson_Process(MergeRobertson b, VectorMat src, Mat dst, VectorFloat times);
void MergeRobertson_Close(MergeRobertson b);

Tonemap Tonemap_Create();
Tonemap Tonemap_CreateWithParams(float gamma);
void Tonemap_Process(Tonemap b, Mat src, Mat dst);
void Tonemap_Close(Tonemap b);

TonemapDrago TonemapDrago_Create();
TonemapDrago TonemapDrago_CreateWithParams(float gamma, float saturation, float bias);
void TonemapDrago_Process(TonemapDrago b, Mat src, Mat dst);
void TonemapDrago_Close(TonemapDrago b);

TonemapMantiuk TonemapMantiuk_Create();
TonemapMantiuk TonemapMantiuk_CreateWithParams(float gamma, float scale, float saturation);
void TonemapMantiuk_Process(TonemapMantiuk b, Mat src, Mat dst);
void TonemapMantiuk_Close(TonemapMantiuk b);

TonemapReinhard TonemapReinhard_Create();
TonemapReinhard TonemapReinhard_CreateWithParams(float gamma, float intensity, float light_adapt, float color_adapt);
void TonemapReinhard_Process(TonemapReinhard b, Mat src, Mat dst);
void TonemapReinhard_Close(TonemapReinhard b);

AlignMTB AlignMTB_Create();
AlignMTB AlignMTB_CreateWithParams(int max_bits, int exclude_range, bool cut);
void AlignMTB_Process(AlignMTB b, struct Mats src, struct Mats *dst);
void AlignMTB_Close(AlignMTB b);

void DetailEnhance(Mat src, Mat dst, float sigma_s, float sigma_r);
void EdgePreservingFilter(Mat src, Mat dst, int filter, float sigma_s, float sigma_r);
void PencilSketch(Mat src, Mat dst1, Mat dst2, float sigma_s, float sigma_r, float shade_factor);
void Stylization(Mat src, Mat dst, float sigma_s, float sigma_r);

#ifdef __cplusplus
}
#endif

#endif //_OPENCV3_PHOTO_H
