#include <segmentation/SegmentationGMMGrabCut.h>
#include <boost/foreach.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

void SegmentationGMMGrabCut::set_strategy(const int strategy) {
    if(strategy != EXTRACT_ALL && strategy != EXTRACT_CLOSEST_TO_CENTER && strategy != EXTRACT_LARGEST)
        this->strategy = EXTRACT_ALL;
    else
        this->strategy = strategy;
}

void SegmentationGMMGrabCut::estimate_pbg(const cv::Mat& img, const cv::Mat& mask) {
    using namespace cv;
    std::vector<double> prs;
    prs.reserve(img.cols * img.rows);

    for(int i = 0; i < img.rows; i++) {
        for(int j = 0; j < img.cols; j++) {
            cv::Mat z(3, 1, CV_64FC1);
            for(int c = 0; c < 3; c++) {
                z.at<double>(c, 0) = (double) img.at<cv::Vec3b>(i, j)[c];
            }
            double p = get_gc_prob(z);
            prs.push_back(p);
        }
    }

    {
        const unsigned int N = prs.size() * (estimate_pbg_sure_percent / 100.0);
        std::nth_element(prs.begin(), prs.begin() + N, prs.end());
        sbg = *(prs.begin() + N);
    }
    {
        const unsigned int N = prs.size() * (estimate_pbg_prob_percent / 100.0);
        std::nth_element(prs.begin(), prs.begin() + N, prs.end());
        pbg = *(prs.begin() + N);
    }
}

bool SegmentationGMMGrabCut::segment(const cv::Mat& img, cv::Mat& segmented_mask, const cv::Mat& mask) {
    using namespace cv;
    using namespace std;
    if(!learned)
        return false;

    Mat gc_mask;
    gc_mask.create(img.size(), CV_8UC1);

    for(int i = 0; i < img.rows; i++) {
        #pragma omp parallel for
        for(int j = 0; j < img.cols; j++) {
            cv::Mat z(3, 1, CV_64FC1);
            for(int c = 0; c < 3; c++) {
                z.at<double>(c, 0) = (double) img.at<cv::Vec3b>(i, j)[c];
            }
            if(mask.empty()) {
                double p = get_gc_prob(z);
                gc_mask.at<uchar>(i, j) = get_gc_class(p, sbg, pbg);
            } else {
                if(mask.at<uchar>(i, j) != 0) {
                    double p = get_gc_prob(z);
                    gc_mask.at<uchar>(i, j) = get_gc_class(p, sbg, pbg);
                } else {
                    gc_mask.at<uchar>(i, j) = GC_BGD;

                }
            }
        }
    }

    if(show_gmm) {
        cv::Mat gmm;
        gmm = gc_mask * 63;
        imshow("gmm", gmm);
    }

    segmented_mask.create(img.size(), CV_8UC1);
    segmented_mask.setTo(Scalar::all(0));
    int nonZero = countNonZero(gc_mask) ;
    if((nonZero == gc_mask.cols * gc_mask.rows) || (nonZero == 0)) //There is no SURE_BACKROUND set
        return false;
    Mat bgdModel, fgdModel;
    try {
        grabCut(img, gc_mask, Rect(), bgdModel, fgdModel, grabcut_iterations, GC_INIT_WITH_MASK);
    } catch(...) {
        return false;
    }
    Mat bin_mask(gc_mask.size(), CV_8UC1, Scalar(0));
    bin_mask.setTo(Scalar(255), gc_mask & 1);

    if(strategy == EXTRACT_ALL) {
        segmented_mask = bin_mask;
        return true;
    }

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(bin_mask, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    int selected_contour_index = -1;

    if(strategy == EXTRACT_LARGEST) {
        double largest_area = 0.0;
        for(unsigned int i = 0; i < contours.size(); i++) {
            double a = contourArea(contours[i], false);
            if(a >= largest_area) {
                largest_area = a;
                selected_contour_index = i;
            }
        }
    }

    if(strategy == EXTRACT_CLOSEST_TO_CENTER) {
        double closest_dist = 100000.0;
        Point2f imgc = Point2f(bin_mask.rows / 2.0, bin_mask.cols / 2.0);
        for(int i = 0; i < contours.size(); i++) {
            Moments mu = moments(contours[i], true);
            Point2f c = Point2f(mu.m10 / mu.m00 , mu.m01 / mu.m00);
            double d = norm(c - imgc);
            double a = contourArea(contours[i], false);
            if(d < closest_dist && a > min_contour_area) {
                closest_dist = d;
                selected_contour_index = i;
            }
        }
    }

    if(selected_contour_index == -1) { //No countour found
        return false;
    }
    segmented_mask.setTo(Scalar::all(0));
    drawContours(segmented_mask, contours, selected_contour_index, Scalar::all(255), CV_FILLED, 8, hierarchy);
}

double SegmentationGMMGrabCut::gaussian_pdf(const cv::Mat& x, const cv::Mat& mu, const cv::Mat& cov) {
    static const double GAUSS_PI_CONST = pow(2 * M_PI, -mu.rows / 2.0);

    double det = cv::determinant(cov);
    cv::Mat icov = cov.inv();

    cv::Mat v;
    v = x - mu;
    double exponent = ((double) cv::Mat(v.t() * (icov * v)).at<double>(0, 0)) * (-0.5);

    if(isnan(exponent))
        return 0.0;
    double nu = GAUSS_PI_CONST / sqrt(det);
    double ret = exp(exponent) * nu;
    return ret;
}


double SegmentationGMMGrabCut::get_gc_prob(const cv::Mat& z) {
    double p_z = 0.0;
    for(unsigned int i = 0 ; i < weights.size(); ++i) {
        cv::Mat mu = means[i];
        cv::Mat cov = covs[i];
        cv::Mat w = weights[i];

        p_z += w.at<double>(0, 0) * gaussian_pdf(z, mu, cov);
    }
    return p_z;
}

uchar SegmentationGMMGrabCut::get_gc_class(const cv::Mat& z, double PSB, double PPB, double PSF) {
    double p_z = get_gc_prob(z);
    return get_gc_class(p_z, PSB, PPB, PSF);
}

uchar SegmentationGMMGrabCut::get_gc_class(double p, double PSB, double PPB, double PSF) {
    double p_z = p;
    if(p_z > PSB)
        return cv::GC_BGD;
    else if(p_z > PPB)
        return cv::GC_PR_BGD;
    else if(p_z < PSF)
        return cv::GC_FGD;
    else
        return cv::GC_PR_FGD;

}

void SegmentationGMMGrabCut::learn(const cv::Mat& img, const cv::Mat& mask) {
    std::vector<Cluster> clusters;
    clusterize(img, mask, clusters);
    weights.clear();
    means.clear();
    covs.clear();

    double tot_weight = 0;
    BOOST_FOREACH(Cluster & c, clusters) {
        tot_weight += c.pixels.rows;
    }

    BOOST_FOREACH(Cluster & c, clusters) {
        using namespace cv;

        Mat w = Mat(1, 1, CV_64FC1, c.pixels.rows / tot_weight);
        Mat u = Mat::zeros(3, 1, CV_64FC1);
        Mat cov = Mat::zeros(3, 3, CV_64FC1);

        for(unsigned int  i = 0 ; i < c.pixels.rows; ++i) {
            Mat z, row = c.pixels.row(i);
            Mat(row.t()).convertTo(z, CV_64FC1);
            u += z;

            Mat v = (z - c.get_mean());
            cov += (v * v.t());
        }
        u = u / c.pixels.rows;
        cov = cov / c.pixels.rows;

        weights.push_back(w);
        means.push_back(u);
        covs.push_back(cov);
    }
    learned = true;
}

void SegmentationGMMGrabCut::clusterize(const cv::Mat& img, const cv::Mat& mask, std::vector< Cluster >& clusters) {
    clusters.resize(1);
    clusters[0].from_img(img, mask);

    for(int k = 0; k < K_MAX - 1; k++) {
        int max_c = 0;
        //find largest cluster in term of eigen values
        for(unsigned int i = 1 ; i < clusters.size(); ++i) {
            if(clusters[i].get_eig_value() > clusters[max_c].get_eig_value()) {
                max_c = i;
            }
        }

        Cluster a, b;
        clusters[max_c].split(a, b);
        clusters.push_back(a);
        clusters.push_back(b);
        clusters.erase(clusters.begin() + max_c);
    }

}

bool SegmentationGMMGrabCut::load(const std::string& filename) {
    std::ifstream ifs(filename.c_str(), std::ios::in | std::ios::binary);
    if(!ifs) {
        return false;
    }
    {
        boost::archive::binary_iarchive ia(ifs);
        ia >> *this;
    }
    ifs.close();
    return true;
}

bool SegmentationGMMGrabCut::save(const std::string& filename) {
    std::ofstream ofs(filename.c_str(), std::ios::out | std::ios::binary);
    if(!ofs) {
        return false;
    }
    {
        boost::archive::binary_oarchive oa(ofs);
        oa << *this;
    }
    ofs.close();
    return true;
}
