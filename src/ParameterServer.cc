#include "ParameterServer.h"

namespace ORB_SLAM2 
{
ParameterServer* ParameterServer::_instance=NULL;

ParameterServer* ParameterServer::instance()
{
  if(_instance==NULL)
    _instance=new ParameterServer();
  return _instance;
}
ParameterServer::ParameterServer()
{

}

bool ParameterServer::setFilePath(const std::string filename)
{
  return loadParam(filename);
}
bool ParameterServer::loadParam(const std::string filename)
{
  std::string str;
  cv::FileNode node;
  cv::FileStorage fs(filename.c_str(),cv::FileStorage::READ);
  if(!fs.isOpened())
  {
    return false;
  }
  //TODO:append parameter
setParam("Camera.fx",fs["Camera.fx"]);
setParam("Camera.fy",fs["Camera.fy"]);
setParam("Camera.cx",fs["Camera.cx"]);
setParam("Camera.cy",fs["Camera.cy"]);
setParam("Camera.k1",fs["Camera.k1"]);
setParam("Camera.k2",fs["Camera.k2"]);
setParam("Camera.p1",fs["Camera.p1"]);
setParam("Camera.p2",fs["Camera.p2"]);
setParam("Camera.k3",fs["Camera.k3"]);
setParam("Camera.width",fs["Camera.width"]);
setParam("Camera.height",fs["Camera.height"]);
setParam("Camera.fps",fs["Camera.fps"]);
setParam("Camera.bf",fs["Camera.bf"]);
setParam("Camera.RGB",fs["Camera.RGB"]);
setParam("ThDepth",fs["ThDepth"]);
setParam("DepthMapFactor",fs["DepthMapFactor"]);
setParam("ORBextractor.nFeatures",fs["ORBextractor.nFeatures"]);
setParam("ORBextractor.scaleFactor",fs["ORBextractor.scaleFactor"]);
setParam("ORBextractor.nLevels",fs["ORBextractor.nLevels"]);
setParam("ORBextractor.iniThFAST",fs["ORBextractor.iniThFAST"]);
setParam("ORBextractor.minThFAST",fs["ORBextractor.minThFAST"]);
setParam("Tracking.StereoInitializationTh",fs["Tracking.StereoInitializationTh"]);
setParam("Tracking.TrackReferenceKeyFrameRatio",fs["Tracking.TrackReferenceKeyFrameRatio"]);
setParam("Tracking.TrackReferenceKeyFrameMatches",fs["Tracking.TrackReferenceKeyFrameMatches"]);
setParam("Tracking.TrackWithMotionModelRatio",fs["Tracking.TrackWithMotionModelRatio"]);
setParam("Tracking.TrackWithMotionModelTh",fs["Tracking.TrackWithMotionModelTh"]);
setParam("Tracking.TrackWithMotionModelMatches",fs["Tracking.TrackWithMotionModelMatches"]);
setParam("Tracking.TrackCameraPoseInliers",fs["Tracking.TrackCameraPoseInliers"]);
setParam("Tracking.TrackLocalMapInliersRelocalization",fs["Tracking.TrackLocalMapInliersRelocalization"]);
setParam("Tracking.TrackLocalMapInliers",fs["Tracking.TrackLocalMapInliers"]);
setParam("Tracking.TrackLocalKeyFramesNum",fs["Tracking.TrackLocalKeyFramesNum"]);
setParam("Tracking.SearchLocalPointsRatio",fs["Tracking.SearchLocalPointsRatio"]);
setParam("Tracking.SearchLocalPointsTh",fs["Tracking.SearchLocalPointsTh"]);
setParam("Tracking.SearchLocalPointsThRelocalization",fs["Tracking.SearchLocalPointsThRelocalization"]);
setParam("Tracking.SearchLocalPointsViewLimit",fs["Tracking.SearchLocalPointsViewLimit"]);
setParam("Tracking.RelocalizationBowRatio",fs["Tracking.RelocalizationBowRatio"]);
setParam("Tracking.RelocalizationBowMatches",fs["Tracking.RelocalizationBowMatches"]);
setParam("Tracking.RelocalizationProjectionRatio",fs["Tracking.RelocalizationProjectionRatio"]);
setParam("Tracking.RelocalizationProjectionLow",fs["Tracking.RelocalizationProjectionLow"]);
setParam("Tracking.RelocalizationProjectionMid",fs["Tracking.RelocalizationProjectionMid"]);
setParam("Tracking.RelocalizationProjectionHigh",fs["Tracking.RelocalizationProjectionHigh"]);
setParam("Tracking.NeedNewKeyFrameTh",fs["Tracking.NeedNewKeyFrameTh"]);
setParam("Tracking.NeedNewKeyFrameThLow",fs["Tracking.NeedNewKeyFrameThLow"]);
setParam("Tracking.NeedNewKeyFrameRefRatio",fs["Tracking.NeedNewKeyFrameRefRatio"]);
setParam("Tracking.NeedNewKeyFrameRefRatioLow",fs["Tracking.NeedNewKeyFrameRefRatioLow"]);
setParam("Tracking.ResetFramesNum",fs["Tracking.ResetFramesNum"]);
setParam("Frame.RowsOfGrid",fs["Frame.RowsOfGrid"]);
setParam("Frame.ColssOfGrid",fs["Frame.ColssOfGrid"]);
setParam("LocalMapping.MapPointsRatio",fs["LocalMapping.MapPointsRatio"]);
setParam("LocalMapping.MapPointsTh",fs["LocalMapping.MapPointsTh"]);
setParam("LocalMapping.CreateNewMapPointsKF",fs["LocalMapping.CreateNewMapPointsKF"]);
setParam("LocalMapping.SearchForTriangulationRatio",fs["LocalMapping.SearchForTriangulationRatio"]);
setParam("LocalMapping.SearchInNeighborsKF",fs["LocalMapping.SearchInNeighborsKF"]);
setParam("LocalMapping.SearchInNeighborsRatio",fs["LocalMapping.SearchInNeighborsRatio"]);
setParam("LocalMapping.KeyFramesRatio",fs["LocalMapping.KeyFramesRatio"]);
setParam("LocalMapping.KeyFramesTh",fs["LocalMapping.KeyFramesTh"]);
setParam("LoopClosing.CovisibilityConsistencyTh",fs["LoopClosing.CovisibilityConsistencyTh"]);
setParam("LoopClosing.ComputeSim3Ratio",fs["LoopClosing.ComputeSim3Ratio"]);
setParam("LoopClosing.SearchByBoWTh",fs["LoopClosing.SearchByBoWTh"]);
setParam("LoopClosing.SearchBySim3Th",fs["LoopClosing.SearchBySim3Th"]);
setParam("LoopClosing.SearchByProjectionTh",fs["LoopClosing.SearchByProjectionTh"]);
setParam("LoopClosing.SearchAndFuseRatio",fs["LoopClosing.SearchAndFuseRatio"]);
setParam("LoopClosing.SearchAndFuseTh",fs["LoopClosing.SearchAndFuseTh"]);
  return true;
}

}