#include <opencv2/opencv.hpp>

int main(void)
{
	cv::Mat src1, src2;

  src1 = cv::imread("../picture/left.jpeg");
  src2 = cv::imread("../picture/mid.jpeg");

	cv::cvtColor(src1, src1, cv::COLOR_BGR2GRAY);
	cv::cvtColor(src2, src2, cv::COLOR_BGR2GRAY);
  cv::resize(src1, src1, cv::Size(), 0.3, 0.3);
  cv::resize(src2, src2, cv::Size(), 0.3, 0.3);

	// 特徴点検出アルゴリズムの選択
	// # ORB::create(検出特徴点数, scale factor, ...)
	cv::Ptr<cv::ORB>  orb = cv::ORB::create(500);

	// 検出したキーポイント（特徴点）を格納する配列
	std::vector<cv::KeyPoint> key1, key2;

	// キーポイントの検出
	orb->detect(src1, key1);
	orb->detect(src2, key2);

	// 特徴量記述の計算
	cv::Mat des1, des2;
	orb->compute(src1, key1, des1);
	orb->compute(src2, key2, des2);

	// 特徴点マッチングアルゴリズムの選択
	cv::Ptr<cv::DescriptorMatcher> hamming = cv::DescriptorMatcher::create("BruteForce-Hamming");

	// 特徴点マッチング
	// # 特徴量記述des1とdes2のマッチングを行い、結果をmatchへ書き込む
	std::vector<cv::DMatch> match;
	hamming->match(des1, des2, match);

	// 特徴量距離の小さい順にソートする（選択ソート）
	for (int i = 0; i < match.size() - 1; i++) {
		double min = match[i].distance;
		int n = i;
		for (int j = i + 1; j < match.size(); j++) {
			if (min > match[j].distance) {
				n = j;
				min = match[j].distance;
			}
		}
		std::swap(match[i], match[n]);
	}

	// 上位50点を残して、残りのマッチング結果を削除する。
	match.erase(match.begin() + 50, match.end());

	//KeyPoint -> Point2d
	std::vector<cv::Point2f> match_point1;
	std::vector<cv::Point2f> match_point2;
	for (int i = 0; i < match.size() - 1; i++) {
		match_point1.push_back(key1[match[i].queryIdx].pt);
		match_point2.push_back(key2[match[i].trainIdx].pt);
	}


	// 特徴点マッチングの結果を画像化する
	cv::Mat match_img;
	cv::drawMatches(src1, key1, src2, key2, match, match_img);

	// findHomography関数を用いて射影変換行列を算出
	cv::Mat homography,homo_img;
	homography = cv::findHomography(match_point1, match_point2, CV_RANSAC);
	cv::warpPerspective(src1, homo_img, homography, src1.size(), cv::INTER_LINEAR);

	//差分計算
	cv::Mat diff1,diff2;
	cv::absdiff(src1, src2, diff1);
	cv::absdiff(homo_img, src2, diff2);

	cv::imshow("画像1", src1);
	cv::imshow("画像2", src2);
	//cv::imshow("マッチング結果", match_img);
	//cv::imshow("射影変換", homo_img);
	cv::imshow("差分1", diff1);
	cv::imshow("差分2", diff2);
	cv::waitKey();

	return 0;
}
