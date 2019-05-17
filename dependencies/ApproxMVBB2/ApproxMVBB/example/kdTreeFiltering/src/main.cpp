// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz
//  (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#include <fstream>
#include <iostream>

#include "ApproxMVBB/Config/Config.hpp"
#include ApproxMVBB_TypeDefs_INCLUDE_FILE
#include "ApproxMVBB/KdTree.hpp"

#ifdef ApproxMVBB_SUPPORT_XML
#include "ApproxMVBB/KdTreeXml.hpp"
#endif

// Not part of library:
#include "CPUTimer.hpp"

ApproxMVBB_DEFINE_MATRIX_TYPES ApproxMVBB_DEFINE_POINTS_CONFIG_TYPES

    // Read in file
    Vector3List
    getPointsFromFile3D(std::string filePath)
{
    std::ifstream file;           // creates stream myFile
    file.open(filePath.c_str());  // opens .txt file

    if (!file.is_open())
    {  // check file is open, quit if not
        ApproxMVBB_ERRORMSG("Could not open file: " << filePath)
    }

    PREC a, b, c;
    Vector3List v;
    while (file.good())
    {
        file >> a >> b >> c;
        v.emplace_back(a, b, c);
    }
    file.close();
    return v;
}

/** Special point type with id*/
struct MyPoint
{
    ApproxMVBB_DEFINE_MATRIX_TYPES Vector3* m_p;
    unsigned int m_id;
};
/** Special point getter */
struct MyPointGetter
{
    ApproxMVBB_DEFINE_MATRIX_TYPES static const Vector3& get(const MyPoint& p)
    {
        return *(p.m_p);
    }
    static Vector3& get(MyPoint& p)
    {
        return *(p.m_p);
    }
};

/** Dont do this in header files!!, here in example thats ok :-) */
using namespace ApproxMVBB;

void doKdTree(std::string file)
{
    auto points = getPointsFromFile3D(file);
    std::cout << "Loaded: " << points.size() << " points " << std::endl;

    AABB3d aabb;  // bounding box

    using PointDataTraits = KdTree::DefaultPointDataTraits<3, Vector3, MyPoint, MyPointGetter>;

    /** KdTree example with exotic point traits */
    {
        std::cout << "KDTree::  Exotic point traits , Vector3* +  id, start: =====" << std::endl;
        using Tree               = KdTree::Tree<KdTree::TreeTraits<KdTree::PointData<PointDataTraits>>>;
        using SplitHeuristicType = Tree::SplitHeuristicType;
        using NodeDataType       = Tree::NodeDataType;
        // static const unsigned int Dimension = NodeDataType::Dimension;
        using PointListType = NodeDataType::PointListType;

        using PointListType = NodeDataType::PointListType;

        PointListType t;

        for (unsigned int i = 0; i < points.size(); ++i)
        {
            t.push_back(MyPoint{&points[i], i});
            aabb += points[i];
        }

        // Make kdTree;
        Tree tree;

        typename SplitHeuristicType::QualityEvaluator e(0.0,  /* splitratio (maximized by MidPoint) */
                                                        2.0,  /* pointratio (maximized by MEDIAN)*/
                                                        1.0); /* extentratio (maximized by MidPoint)*/

        PREC minExtent              = 0.001;  // box extents are bigger than this!
        PREC allowSplitAboveNPoints = 2;
        tree.initSplitHeuristic(
            std::initializer_list<SplitHeuristicType::Method>{//            SplitHeuristicType::Method::MEDIAN,
                                                              //            SplitHeuristicType::Method::GEOMETRIC_MEAN,
                                                              SplitHeuristicType::Method::MIDPOINT},
            allowSplitAboveNPoints,
            minExtent,
            SplitHeuristicType::SearchCriteria::FIND_FIRST,
            e,
            0.0,
            0.0,
            0.1);

        auto rootData = std::unique_ptr<NodeDataType>(new NodeDataType(t.begin(), t.end()));

        START_TIMER(start)
        tree.build(aabb, std::move(rootData), 500 /*max tree depth*/, 500000 /*max leafs*/);
        STOP_TIMER_MILLI(count, start)
        std::cout << "KdTree build took: " << count << "ms." << std::endl;

        // auto list = tree.buildLeafNeighboursAutomatic();
        std::cout << tree.getStatisticsString() << std::endl;

#ifdef ApproxMVBB_SUPPORT_XML
        std::string file = "KdTreeResults.xml";
        std::cout << "Saving KdTree XML to: " << file << std::endl;

        pugi::xml_document dataXML;
        KdTree::XML::appendToXML(tree, dataXML);
        dataXML.save_file(file.c_str(), "    ");
#endif
    }

    /** KdTree example with simple point traits */
    {
        std::cout << "KDTree:: Simple point traits , Vector3 only , start: =====" << std::endl;

        using PointDataTraits    = KdTree::DefaultPointDataTraits<3, Vector3, Vector3>;
        using Tree               = KdTree::Tree<KdTree::TreeTraits<KdTree::PointData<PointDataTraits>>>;
        using SplitHeuristicType = Tree::SplitHeuristicType;
        using NodeDataType       = Tree::NodeDataType;
        // static const unsigned int Dimension = NodeDataType::Dimension;
        using PointListType = NodeDataType::PointListType;

        using PointListType = NodeDataType::PointListType;

        PointListType t;

        // Make kdTree;
        Tree tree;

        typename SplitHeuristicType::QualityEvaluator e(0.0,  /* splitratio (maximized by MidPoint) */
                                                        2.0,  /* pointratio (maximized by MEDIAN)*/
                                                        1.0); /* extentratio (maximized by MidPoint)*/

        PREC minExtent              = 0.01;  // box extents are bigger than this!
        PREC allowSplitAboveNPoints = 10;
        tree.initSplitHeuristic(
            std::initializer_list<SplitHeuristicType::Method>{SplitHeuristicType::Method::MEDIAN,
                                                              SplitHeuristicType::Method::GEOMETRIC_MEAN,
                                                              SplitHeuristicType::Method::MIDPOINT},
            allowSplitAboveNPoints,
            minExtent,
            SplitHeuristicType::SearchCriteria::FIND_BEST,
            e,
            0.0,
            0.0,
            0.1);

        auto rootData = std::unique_ptr<NodeDataType>(new NodeDataType(points.begin(), points.end()));

        START_TIMER(start)
        tree.build(aabb, std::move(rootData), 500 /*max tree depth*/, 600 /*max leafs*/);
        STOP_TIMER_MILLI(count, start)

        auto list = tree.buildLeafNeighboursAutomatic();

        std::cout << "KdTree build took: " << count << "ms." << std::endl;
        std::cout << tree.getStatisticsString() << std::endl;
    }

    /** KdTree example with simple point traits */
    {
        std::cout << "KDTreeOutlierFiltering:: Simple point traits , Vector3 only "
                     ", start: ====="
                  << std::endl;
        using PointDataTraits = KdTree::DefaultPointDataTraits<3, Vector3, Vector3>;

        int kNMean     = 30;
        int stdDevMult = 5;
        KdTree::NearestNeighbourFilter<PointDataTraits> f(kNMean, stdDevMult, 10);
        std::cout << "Input points: " << points.size() << std::endl;
        decltype(points) output;
        f.filter(points, aabb, output, true);
        std::cout << "Classified " << output.size() << " points as outliers." << std::endl;
        std::cout << "Filtered by collecting for each point " << kNMean
                  << " nearst neighbours "
                     ", averaging these nearst neighbour distances for each point which "
                     "results in a histogram (mean, stdDeviation)"
                     " for the distribution of the mean nearest neighbour distance, "
                     " which is used to determine which points have nearest neighbour "
                     "distance >= then mean + "
                  << stdDevMult << " * stdDeviation which classifies the points as outlier points." << std::endl;
    }
}

int main(int, char**)
{
    doKdTree("./Bunny.txt");
    // doKdTree("./Lucy.txt");
    return 0;
}
