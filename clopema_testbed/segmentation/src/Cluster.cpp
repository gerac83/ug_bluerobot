#include <segmentation/Cluster.h>

void Cluster::split(Cluster& c1, Cluster& c2) {
    using namespace cv;
    Mat v = get_eig_vector().clone();
    Mat m = get_mean().clone();
    double thresh = Mat(v.t() * m).at<double>(0, 0);
    Mat tmp = pixels.clone();
    c1.pixels = Mat(0, tmp.cols, tmp.type());
    c2.pixels = Mat(0, tmp.cols, tmp.type());

    for(int i = 0; i < tmp.rows; i++) {
        Mat z =  tmp.row(i).t();
        double val = Mat(v.t() * z).at<double>(0, 0);
        if(val <= thresh) {
            c1.pixels.push_back(tmp.row(i).clone());
        } else {
            c2.pixels.push_back(tmp.row(i).clone());
        }
    }
}

cv::Mat Cluster::get_eig_vector() {
    if(!computed)
        compute_PCA();
    return eig_vector;

}

cv::Mat Cluster::get_mean() {
    if(!computed)
        compute_PCA();
    return mean;

}


double Cluster::get_eig_value()  {
    if(!computed)
        compute_PCA();
    return eig_value;
}

void Cluster::convert_pixels_to_row(const cv::Mat& img, cv::Mat& data, const cv::Mat& mask) {
    cv::Mat m(img.size(), CV_8UC1, cv::Scalar(1));
    if(!mask.empty())
        mask.copyTo(m);
    cv::Mat tmp(img.rows * img.cols, 3, CV_64FC1);
    int k = 0;
    for(int i = 0; i < img.rows; i++) {
        for(int j = 0; j < img.cols; j++) {
            if(m.at<uchar>(i, j) != 0) {
                for(int c = 0; c < 3; c++) {
                    tmp.at<double>(k, c) = (double) img.at<cv::Vec3b>(i, j)[c];
                }
                k++;
            }
        }
    }
    
    data  = tmp.rowRange(0, k-1);   
}

void Cluster::from_img(const cv::Mat& img, const cv::Mat& mask)  {
    Cluster::convert_pixels_to_row(img, pixels, mask);
    computed = false;
}

void Cluster::compute_PCA()  {
    using namespace cv;
    if(computed)
        return;
    PCA pca(pixels, Mat(), CV_PCA_DATA_AS_ROW);

    mean = pca.mean.row(0).t();
    eig_value = -1;
    for(int i = 0; i < 2; i++) {
        double eig_tmp = pca.eigenvalues.at<double>(0, i);
        if(eig_tmp < eig_value)
            continue;
        eig_value = eig_tmp;
        eig_vector = pca.eigenvectors.row(i).t();
    }

    computed = true;
}
