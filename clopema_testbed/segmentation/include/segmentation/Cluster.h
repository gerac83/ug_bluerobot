/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 05/16/14
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *  Institute: Czech Technical University in Prague
 *    Details: Cluster class representation
 */

#ifndef CLUSTER_H
#define CLUSTER_H

#include <opencv2/imgproc/imgproc.hpp>

class Cluster {
    public:
        Cluster() : computed(false) {}
        ~Cluster() {}

        /** \brief Initialize cluster from the RGB image */
        void from_img(const cv::Mat& img, const cv::Mat& mask = cv::Mat());
        /** \brief Split into two clusters c1 and c2 */
        void split(Cluster& c1, Cluster& c2);

        /** \brief Get eigen value for the cluster */
        double get_eig_value();
        /** \brief Get eigen vector for the cluster */
        cv::Mat get_eig_vector();
        /** \brief Get mean value for the cluster */
        cv::Mat get_mean();

        /** \brief Convert RGB image to the vector of RGBs data (i.e. MxNx3 to the M*Nx3x1) */
        static void convert_pixels_to_row(const cv::Mat& img, cv::Mat& data, const cv::Mat& mask = cv::Mat());

    private:
        /** \brief Compute PCA is called when internal matrix 'pixels' was modified */
        void compute_PCA();

    public:
        cv::Mat pixels;
    private:
        cv::Mat eig_vector, mean;
        double eig_value;
        bool computed;
};

#endif //CLUSTER
