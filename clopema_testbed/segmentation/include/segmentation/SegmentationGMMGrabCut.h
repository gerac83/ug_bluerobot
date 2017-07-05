/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 05/16/14
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 *    Details: Segmentation using GMM and GrabCut Class
 *
 * ---------------------------------------------------------
 *      Segmentation works as follow:
 *              1) In learn step, Gaussian Mixture Model (GMM) is constructed using the template image which consist of the background colors only.
 *                      a) seg.learn(template);
 *              2) Probabilities which defines the initial values of each pixel is learned from the tamplate image (background colors).
 *                      a) seg.estimate_pbg(template);
 *                 The probabilities are learn as follow:
 *                      a) prob = compute probabilities for each pixel and store them in the vector
 *                      b) sort probabilities 'prob' from smallest to largest
 *                      c) pick the N-th value from prob and set it as SURE_BACKGROUND threshold,
 *                         where N is picked based on the parameter 'estimate_pbg_sure_percent' (10% by the default),
 *                         meaning that 90% of the template is sure bacgkround.
 *                         Additionally PROBABLE_BACKGROUND is set based on the parameter 'estimate_pbg_prob_percent' (.01% by the default),
 *                         meaning that 99.99% of template stands for probable background.
 *              3) Segment the image based on the estimated probabilities. Roughly segmented image (based on the GMM) is then used as the input to
 *                 the GrabCut algorithm, result is then posporocessed based on the strategy.
 *                      a) Strategy EXTRACT_ALL (default) return the segmented mask
 *                      b) Strategy EXTRACT_LARGEST return the largest contour (based on the area)
 *                      c) Strategy EXTRACT_CLOSEST_TO_CENTER return the contour which center is closest to the image center and the area is larger then 'min_contour_area'
 *
 *                      a) seg.segment(image, out_mask);
 *
 * ----------------------------------------------------------
 *      How to start using the segmentation:
 *              a) Take picture of the color you want to segment out from the other images (the image of the table desk for example).
 *                 No other color should be inside the this 'template' image, otherwise learning will not be sucesfull.
 *              b) Call functions seg.learn(template); and seg.estimate_pbg(template); where seg stands for the instance of the class.
 *              c) Use function seg.segment(img, out_mask); to extract the binary mask from the image 'img';
 */

#ifndef SEGMENTATIONGMMGRABCUT_H
#define SEGMENTATIONGMMGRABCUT_H

#include <segmentation/Cluster.h>
#include <segmentation/cvmat_serialization.h>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/access.hpp>

class SegmentationGMMGrabCut {
    public:
        enum SegmentationStrategy {EXTRACT_ALL = 0, EXTRACT_LARGEST = 1, EXTRACT_CLOSEST_TO_CENTER = 2};

    public:
        /** \brief Construct Segmentation GMM GrabCut Class and set the default values */
        SegmentationGMMGrabCut(int k = 2): K_MAX(k), learned(false), pbg(1e-5), show_gmm(false) {
            estimate_pbg_sure_percent = 10;
            estimate_pbg_prob_percent = 0.01;
            grabcut_iterations = 3;
            strategy = EXTRACT_ALL;
            min_contour_area = 100;
        }

        /** \brief Destructor */
        ~SegmentationGMMGrabCut() {}

        /** \brief Learn the segmentation parameters */
        void learn(const cv::Mat& img, const cv::Mat& mask = cv::Mat());

        /** \brief Estimate PBG from the template image
            \details Compute probability from learned GMM and assume that X% is background */
        void estimate_pbg(const cv::Mat& img, const cv::Mat& mask = cv::Mat());

        /** \brief Segment the input image - output mask is stored in segmented matrix */
        bool segment(const cv::Mat& img, cv::Mat& segmented, const cv::Mat& mask = cv::Mat());

        /** \brief Set strategy for segmentation, which contour is returned. Will return all by default. */
        void set_strategy(const int strategy = EXTRACT_ALL);

        /** \brief Save the current model to the file */
        bool save(const std::string& filename);
        /** \brief Load the learned model from the file */
        bool load(const std::string& filename);

        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive& ar, const unsigned int version) {
            ar& K_MAX& learned& covs& means& weights& sbg& pbg& estimate_pbg_prob_percent &
            estimate_pbg_sure_percent& grabcut_iterations& strategy& min_contour_area& show_gmm;
        }
        
    private:
        /** \brief Clusterize template image to the k_MAX clusters */
        void clusterize(const cv::Mat& img, const cv::Mat& mask, std::vector<Cluster>& clusters);

        /** \brief Get GrabCut class based on the pixel z and threshold parameters PSB,PPB,PSF */
        uchar get_gc_class(const cv::Mat& z, double PSB = 1, double PPB = 1, double PSF = -1);
        /** \brief Get GrabCut class based on the probability p and threshold parameters PSB,PPB,PSF */
        uchar get_gc_class(double p, double PSB = 1, double PPB = 1, double PSF = -1);
        /** \brief Get GMM probability of the pixel */
        double get_gc_prob(const cv::Mat& z);

        /** \brief Compute probability density function in state 'x' with gaussian parameters 'mu' and 'cov' */
        double gaussian_pdf(const cv::Mat& x, const cv::Mat& mu, const cv::Mat& cov);

    private:
        /** \brief Number of gaussians in the GMM */
        int K_MAX;
        /** \brief Whether learning was done - checked before the segmentation is done */
        bool learned;

    public:
        /** \brief Learned Gaussian Mixture Model */
        std::vector<cv::Mat> covs, means, weights;
        /** \brief Estimated PBG threshold (probable background, sure background) */
        double sbg, pbg;
        /** \brief How much percent of template is used for SURE_Background [10%] */
        double estimate_pbg_sure_percent;
        /** \brief How much percent of template is used for PROB_Background [0.1%] */
        double estimate_pbg_prob_percent;
        /** \brief Number of grabcut iterations [3] */
        int grabcut_iterations;
        /** \brief Strategy used for segmentation [EXTRACT_ALL], optionally can return only one contour based on strategy */
        int strategy;
        /** \brief Minimal contour area if [EXTRACT_CLOSEST_TO_CENTER] strategy is used [10px] */
        double min_contour_area;

        /** \brief Whether show GMM segmentation - debug purpose only */
        bool show_gmm;

};

#endif //SEGMENTATIONGMMGRABCUT
