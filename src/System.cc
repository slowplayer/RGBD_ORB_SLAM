#include "System.h"


namespace ORB_SLAM2 
{
System::System(const string& strVocFile, const string& strSettingsFile):mbReset(false)
{
  //Load Parameter
  if(!ParameterServer::instance()->setFilePath(strSettingsFile))
  {
    cerr<<"Failed to open setting file at: "<<strSettingsFile<<endl;
    exit(-1);
  }
  //Load ORB Vocabulary
  cout<<"Loading ORB Vocabulary. This could take a while..."<<endl;
  mpVocabulary=new ORBVocabulary();
  if(!mpVocabulary->loadFromTextFile(strVocFile))
  {
    cerr<<"Wrong path to vocabulary."<<endl;
    cerr<<"Failed to open at:"<<strVocFile<<endl;
    exit(-1);
  }
  cout<<"Vocabulary loaded!"<<endl<<endl;
  
  mpKeyFrameDatabase=new KeyFrameDatabase(*mpVocabulary);
  mpMap=new Map();
  
  mpFrameDrawer=new FrameDrawer(mpMap);
  mpMapDrawer=new MapDrawer(mpMap);
  
  mpTracker=new Tracking(this,mpVocabulary,mpFrameDrawer,mpMapDrawer,mpMap,mpKeyFrameDatabase);
  
  mpLocalMapper=new LocalMapping(mpMap);
  mptLocalMapping=new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);
  
  mpLoopCloser=new LoopClosing(mpMap,mpKeyFrameDatabase,mpVocabulary,true);
  mptLoopClosing=new thread(&ORB_SLAM2::LoopClosing::Run,mpLoopCloser);
  
  mpViewer=new Viewer(this,mpFrameDrawer,mpMapDrawer,mpTracker);
  mptViewer=new thread(&Viewer::Run,mpViewer);
  mpTracker->SetViewer(mpViewer);
  
  mpTracker->SetLocalMapper(mpLocalMapper);
  mpTracker->SetLoopClosing(mpLoopCloser);
  
  mpLocalMapper->SetTracker(mpTracker);
  mpLocalMapper->SetLoopCloser(mpLoopCloser);
  
  mpLoopCloser->SetTracker(mpTracker);
  mpLoopCloser->SetLocalMapper(mpLocalMapper);
}
void System::TrackRGBD(const cv::Mat& im, const cv::Mat& depthmap, const double& timestamp)
{
  unique_lock<mutex> lock(mMutexReset);
  if(mbReset)
  {
    mpTracker->Reset();
    mbReset=false;
  }
  
  mpTracker->GrabImageRGBD(im,depthmap,timestamp);
}
void System::Reset()
{
  unique_lock<mutex> lock(mMutexReset);
  mbReset=true;
}
void System::Shutdown()
{
  mpLocalMapper->RequestFinish();
  mpLoopCloser->RequestFinish();
  mpViewer->RequestFinish();
  
  while(!mpLocalMapper->isFinished()||!mpLoopCloser->isFinished()||
    !mpViewer->isFinished()||mpLoopCloser->isRunningGBA())
  {
    usleep(5000);
  }
}
}