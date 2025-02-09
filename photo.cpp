#include "photo.h"

VectorMat VectorMat_Create() {
    return new std::vector<cv::Mat>();
}
VectorMat VectorMat_CreateWithCapacity(uint size) {
    VectorMat m = VectorMat_Create();
    m->reserve(size);
    return m;
}

void VectorMat_PushBack(VectorMat m, Mat mat) {
    m->push_back(*mat);
}

void VectorMat_Free(VectorMat m) {
    delete m;
}


VectorInt VectorInt_Create() {
    return new std::vector<int>();
}
VectorInt VectorInt_CreateWithCapacity(uint size) {
    VectorInt m = VectorInt_Create();
    m->reserve(size);
    return m;
}

void VectorInt_PushBack(VectorInt m, int v) {
    m->push_back(v);
}

void VectorInt_Free(VectorInt m) {
    delete m;
}




void ColorChange(Mat src, Mat mask, Mat dst, float red_mul, float green_mul, float blue_mul) {
    cv::colorChange(*src, *mask, *dst, red_mul, green_mul, blue_mul);
}

void IlluminationChange(Mat src, Mat mask, Mat dst, float alpha, float beta) {
    cv::illuminationChange(*src, *mask, *dst, alpha, beta);
}

void SeamlessClone(Mat src, Mat dst, Mat mask, Point p, Mat blend, int flags) {
    cv::Point pt(p.x, p.y);
    cv::seamlessClone(*src, *dst, *mask, pt, *blend, flags);
}

void TextureFlattening(Mat src, Mat mask, Mat dst, float low_threshold, float high_threshold, int kernel_size) {
    cv::textureFlattening(*src, *mask, *dst, low_threshold, high_threshold, kernel_size);
}


void FastNlMeansDenoisingColoredMulti(	struct Mats src, Mat dst, int imgToDenoiseIndex, int 	temporalWindowSize){
  std::vector<cv::Mat> images;
  for (int i = 0; i < src.length; ++i) {
    images.push_back(*src.mats[i]);
  }
  cv::fastNlMeansDenoisingColoredMulti( images, *dst, imgToDenoiseIndex, 	temporalWindowSize );
}

void FastNlMeansDenoisingColoredMultiWithParams( struct Mats src, Mat dst, int imgToDenoiseIndex, int 	temporalWindowSize, float 	h, float 	hColor, int 	templateWindowSize, int 	searchWindowSize ){
  std::vector<cv::Mat> images;
  for (int i = 0; i < src.length; ++i) {
    images.push_back(*src.mats[i]);
  }
  cv::fastNlMeansDenoisingColoredMulti( images, *dst, imgToDenoiseIndex, 	temporalWindowSize, h, hColor, templateWindowSize, searchWindowSize );
}

MergeMertens MergeMertens_Create() {
  return new cv::Ptr<cv::MergeMertens>(cv::createMergeMertens());
}

MergeMertens MergeMertens_CreateWithParams(float contrast_weight,
                                           float saturation_weight,
                                           float exposure_weight) {
  return new cv::Ptr<cv::MergeMertens>(cv::createMergeMertens(
      contrast_weight, saturation_weight, exposure_weight));
}

void MergeMertens_Close(MergeMertens b) {
  delete b;
}

void MergeMertens_Process(MergeMertens b, VectorMat src, Mat dst) {
  (*b)->process(*src, *dst);
}

MergeDebevec MergeDebevec_Create() {
  return new cv::Ptr<cv::MergeDebevec>(cv::createMergeDebevec());
}

void MergeDebevec_Process(MergeDebevec b, VectorMat src, Mat dst, VectorInt times) {
  (*b)->process(*src, *dst, *times);
}

void MergeDebevec_Close(MergeDebevec b) {
  delete b;
}

MergeRobertson MergeRobertson_Create() {
  return new cv::Ptr<cv::MergeRobertson>(cv::createMergeRobertson());
}

void MergeRobertson_Process(MergeRobertson b, VectorMat src, Mat dst, VectorInt times) {
  (*b)->process(*src, *dst, *times);
}

void MergeRobertson_Close(MergeRobertson b) {
  delete b;
}

Tonemap Tonemap_Create() {
  return new cv::Ptr<cv::Tonemap>(cv::createTonemap());
}

Tonemap Tonemap_CreateWithParams(float gamma) {
  return new cv::Ptr<cv::Tonemap>(cv::createTonemap(gamma));
}

void Tonemap_Process(Tonemap b, Mat src, Mat dst) {
  (*b)->process(*src, *dst);
}

void Tonemap_Close(Tonemap b) {
  delete b;
}

TonemapDrago TonemapDrago_Create() {
  return new cv::Ptr<cv::TonemapDrago>(cv::createTonemapDrago());
}

TonemapDrago TonemapDrago_CreateWithParams(float gamma, float saturation, float bias) {
  return new cv::Ptr<cv::TonemapDrago>(cv::createTonemapDrago(gamma, saturation, bias));
}

void TonemapDrago_Process(TonemapDrago b, Mat src, Mat dst) {
  (*b)->process(*src, *dst);
}

void TonemapDrago_Close(TonemapDrago b) {
  delete b;
}

TonemapMantiuk TonemapMantiuk_Create() {
  return new cv::Ptr<cv::TonemapMantiuk>(cv::createTonemapMantiuk());
}

TonemapMantiuk TonemapMantiuk_CreateWithParams(float gamma, float scale, float saturation) {
  return new cv::Ptr<cv::TonemapMantiuk>(cv::createTonemapMantiuk(gamma, scale, saturation));
}

void TonemapMantiuk_Process(TonemapMantiuk b, Mat src, Mat dst) {
  (*b)->process(*src, *dst);
}

void TonemapMantiuk_Close(TonemapMantiuk b) {
  delete b;
}

TonemapReinhard TonemapReinhard_Create() {
  return new cv::Ptr<cv::TonemapReinhard>(cv::createTonemapReinhard());
}

TonemapReinhard TonemapReinhard_CreateWithParams(float gamma, float intensity, float light_adapt, float color_adapt) {
  return new cv::Ptr<cv::TonemapReinhard>(cv::createTonemapReinhard(gamma, intensity, light_adapt, color_adapt));
}

void TonemapReinhard_Process(TonemapReinhard b, Mat src, Mat dst) {
  (*b)->process(*src, *dst);
}

void TonemapReinhard_Close(TonemapReinhard b) {
  delete b;
}

AlignMTB AlignMTB_Create() {
  return new cv::Ptr<cv::AlignMTB>(cv::createAlignMTB(6,4,false));
}

AlignMTB AlignMTB_CreateWithParams(int max_bits, int exclude_range, bool cut) {
  return new cv::Ptr<cv::AlignMTB>(
      cv::createAlignMTB(max_bits, exclude_range, cut));
}

void AlignMTB_Close(AlignMTB b) { delete b; }

void AlignMTB_Process(AlignMTB b, struct Mats src, struct Mats *dst) {

  std::vector<cv::Mat> srcMats;
  for (int i = 0; i < src.length; ++i) {
    srcMats.push_back(*src.mats[i]);
  }

  std::vector<cv::Mat> dstMats;
  (*b)->process(srcMats, dstMats);

  dst->mats = new Mat[dstMats.size()];
  for (size_t i = 0; i < dstMats.size() ; ++i) {
	dst->mats[i] = new cv::Mat( dstMats[i] );
  }
  dst->length = (int)dstMats.size();
}

void FastNlMeansDenoising(Mat src, Mat dst) {
    cv::fastNlMeansDenoising(*src, *dst);
}

void FastNlMeansDenoisingWithParams(Mat src, Mat dst, float h, int templateWindowSize, int searchWindowSize) {
    cv::fastNlMeansDenoising(*src, *dst, h, templateWindowSize, searchWindowSize);
}

void FastNlMeansDenoisingColored(Mat src, Mat dst) {
    cv::fastNlMeansDenoisingColored(*src, *dst);
}

void FastNlMeansDenoisingColoredWithParams(Mat src, Mat dst, float h, float hColor, int templateWindowSize, int searchWindowSize) {
    cv::fastNlMeansDenoisingColored(*src, *dst, h, hColor, templateWindowSize, searchWindowSize);
}

void EdgePreservingFilter(Mat src, Mat dst, int filter, float sigma_s, float sigma_r) {
    cv::edgePreservingFilter(*src, *dst, filter, sigma_s, sigma_r);
}

void DetailEnhance(Mat src, Mat dst, float sigma_s, float sigma_r) {
    cv::detailEnhance(*src, *dst, sigma_s, sigma_r);
}

void PencilSketch(Mat src, Mat dst1, Mat dst2, float sigma_s, float sigma_r, float shade_factor) {
    cv::pencilSketch(*src, *dst1, *dst2, sigma_s, sigma_r, shade_factor);
}

void Stylization(Mat src, Mat dst, float sigma_s, float sigma_r) {
    cv::stylization(*src, *dst, sigma_s, sigma_r);
}

void PhotoInpaint(Mat src, Mat mask, Mat dst, float inpaint_radius, int algorithm_type) {
    cv::inpaint(*src, *mask, *dst, inpaint_radius, algorithm_type);
}
