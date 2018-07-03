#pragma once
#include<time.h>
#include<iostream>
#include<opencv2/opencv.hpp>
#include<sys/stat.h>
using namespace cv;
using namespace std;

class KeyRecordVideo
{
public:
	KeyRecordVideo() {}
	~KeyRecordVideo() {
		_writer.release();
		if (_isWritingVideo)
		{
			cout << _filename << "结束" << endl;
			_filename = "";
		}

	}
	void listening(int key, Mat mat, string suffix)
	{
		if (_isWritingVideo)
		{
			_writer << mat;
		}
		else {

		}
		//if (key == 'w' || key == 'W') {
		if (key == 'v' || key == 'V') {
			_isWritingVideo = !_isWritingVideo;
			if (_isWritingVideo)
			{
				time_t t = time(0);
				char tmp[64];
				mkdir("video_record", S_IRWXU);
				strftime(tmp, sizeof(tmp), "%Y%m%d%H%M%S", localtime(&t));
				_filename = tmp;
				_filename = "video_record/" + _filename;
				_filename.append(suffix + ".avi");
				_writer.open(_filename, CV_FOURCC('D', 'I', 'V', 'X'), 20, mat.size(), true);
				cout << _filename << " 录制开始" << endl;
			}
			else
			{
				_writer.release();
				cout << _filename << " 录制结束" << endl;
				_filename = "";
			}
		}
	}
private:
	bool _isWritingVideo = false;
	string _filename;
	VideoWriter _writer;

};

class KeyRecordImage
{
public:
	KeyRecordImage() {}
	~KeyRecordImage() {}
	//void listening(int key, Mat mat, string picture_name, string suffix)
	void listening(int key, Mat mat, string suffix)
	{
		//if (key == 'e' || key == 'E') {
		if (key == 'p' || key == 'P') {
			_isWritingImage = true;
			if (_isWritingImage)
			{
				string str = to_string(cnt);
				mkdir("picture_record", S_IRWXU);
				_picname.append("picture_record/" + suffix + str + ".bmp");
				imwrite(_picname, mat);
				cout << "保存成功 ## 图片" << cnt << endl;
				//cout << cnt << endl;
				cnt++;
				_picname = "";
			}
			else
			{
				cout << "保存失败" << endl;
			}
		}
	}
private:
	bool _isWritingImage = false;
	string _picname;
	int cnt = 0;
};

