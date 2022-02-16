#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main(){
  Mat	src;
  src = imread("/home/stl/catkin_ws/src/apu_medication_support/data/nothing_feature_map_2.png");
  if (src.empty() == true) {
    // 画像の中身が空なら終了する
    return 0;
  }

  cv::Point2d pt1,pt2;
  pt1.x = 180;
  pt1.y = 180;
  pt2.x = 300;
  pt2.y = 300;

  Mat rect;
  //-----時点を矩形で囲む-----
  //rect = src.clone();
  //rectangle(rect,pt1,pt2, cv::Scalar(1), 1);
  //cv::imwrite("/home/stl/catkin_ws/src/apu_medication_support/picture/rect.png", rect);

  //時点の切り出し
  rect = Mat(src, Rect(pt1,pt2));
  //imwrite("/home/stl/catkin_ws/src/apu_medication_support/picture/pack_f_1_rect.png", f_1_rect);

  //=====ヒストグラム============================================================
	// 分割レベル
	int binNum = 256;
	int histSize[] = {binNum};

	float value_ranges[] = { 0, 256 };
	const float* ranges[] = { value_ranges };
	MatND hist;
	// 0 番目のチャンネルからヒストグラムを求めます．
	int channels[] = {0};
	int dims = 1; // 入力行列（画像）のチャンネル数

	calcHist( &rect, 1, channels, Mat(), // マスクは利用しません
		hist, dims, histSize, ranges,
		true, // ヒストグラムは一様です
		false );
	double maxVal=0;
	minMaxLoc(hist, 0, &maxVal, 0, 0);

  ofstream ofs("/home/stl/catkin_ws/src/apu_medication_support/data/histgram.csv");
	for( int binId = 0; binId < binNum; binId++ ){
		float binVal = hist.at<float>(binId);
		ofs << binId << "\t" << binVal << endl;
	}
}
