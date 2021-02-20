
#ifdef MACOSX
// This is to overcome a possible bug in Apple's GCC.
#define isnan(x) (x==FP_NAN)
#endif



// inline double GridSlamProcessor::optimize(OrientedPoint &opos,OrientedPoint &refpos,OrientedPoint &newpos,double* refScan,const double* curScan){
// 	double rx = 0;
// 	double ry = 0;
// 	double ra = 0;
// 
// 	refpos.theta = Rad2Deg(refpos.theta);
// 	newpos.theta = Rad2Deg(newpos.theta);
// 	
// 	//old laser center in world
// 	OrientPos p1;
// 	p1.pos.setX(refpos.x);
// 	p1.pos.setY(refpos.y);
// 	p1.angle = refpos.theta;
// 	
// 	//new laser center in world
// 	OrientPos p2;
// 	p2.pos.setX(newpos.x);
// 	p2.pos.setY(newpos.y);
// 	p2.angle = newpos.theta;
// 
// 	//new laser center pos in last laser center frame
// 	World2Local(p1.pos.getX(),p1.pos.getY(),p1.angle,p2.pos.getX(),p2.pos.getY(),p2.angle,rx,ry,ra);
// 	OrientPos p3;
// 	p3.pos.setX(rx);
// 	p3.pos.setY(ry);
// 	p3.angle = ra;
// 
// 	//std::cout<<"befor scanmatch ref pos:"<<rx<<" "<<ry<<" "<<ra<<std::endl;
// 
// 	SLaser scanR;
// 	SLaser scanC;
// 
// 	scanR.stamp_ = 0;
// 	scanR.range_max_ = 20.0;
// 	scanR.range_min_ = 0.01;
// 	scanR.resolution_ = Deg2Rad(0.5);
// 	scanR.start_angle_ = Deg2Rad(-135);
// 	
// 
// 	scanC.stamp_ = 0;
// 	scanC.range_max_ = 20.0;
// 	scanC.range_min_ = 0.01;
// 	scanC.resolution_ = Deg2Rad(0.5);
// 	scanC.start_angle_ = Deg2Rad(-135);
// 
// 	for (int i = 0 ; i < LASER_COUNT; ++i)
// 	{
// 		scanR.data_[i] = *(refScan++);
// 		scanC.data_[i] = *(curScan++);
// 	}
// 
// 	OrientPos npos = PolarScanMatch::ScanMatchFun(&scanR,&scanC,p3);
// 
// 	//std::cout<<"after scanmatch new pos:"<<npos.pos.getX()<<" "<<npos.pos.getY()<<" "<<npos.angle <<std::endl;
// 
// 	//new odom by scan match
// 	Loacal2World(npos.pos.getX(),npos.pos.getY(),npos.angle,refpos.x,refpos.y,refpos.theta,rx,ry,ra);
// 	opos.x = rx;
// 	opos.y = ry;
// 	opos.theta = Deg2Rad(ra);
// 
// 	
// 	return 99;
// }
/**Just scan match every single particle.
If the scan matching fails, the particle gets a default likelihood.*/
inline void GridSlamProcessor::scanMatch(const double* plainReading){
  // sample a new pose from each scan in the reference
  
  double sumScore=0;
  for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
    OrientedPoint corrected;
    double score, l, s;
	//add by wht
	//input particles( map , pos ) and new laser data
	//cal a new pos : corrected
	//get a score
	//std::cout<<"begin optimize:"<<time_diff.total_ms()<<std::endl;
    score=m_matcher.optimize(corrected, it->map, it->pose, plainReading);
	//std::cout<<"before optimize:"<<it->pose.x<<" "<<it->pose.y<<" "<<it->pose.theta<<std::endl;
	//std::cout<<"after optimize:"<<corrected.x<<" "<<corrected.y<<" "<<corrected.theta<<std::endl;
	//std::cout<<"end optimize:"<<time_diff.total_ms()<<std::endl;

	//edit by wht 20150914
	//std::cout<<"begin optimize2:"<<time_diff.total_ms()<<std::endl;
	OrientedPoint corrected2;
	//optimize(corrected2,it->previousPose,it->pose,it->previous_scan,plainReading);
	//score = optimize(corrected2,it->previousPose,it->pose,it->previous_scan,plainReading);
	//std::cout<<"end optimize2:"<<time_diff.total_ms()<<std::endl;

	//std::cout<<"old optimize x y th:"<<corrected.x<<" "<<corrected.y<<" "<<corrected.theta<<std::endl;
	//std::cout<<"new optimize x y th:"<<corrected2.x<<" "<<corrected2.y<<" "<<corrected2.theta<<std::endl;
	//end edit
	
	
    //    it->pose=corrected;
    if (score>m_minimumScore){
      it->pose=corrected;
	  //it->pose = corrected2;
    } else {
	if (m_infoStream){
	  m_infoStream << "Scan Matching Failed, using odometry. Likelihood=" << l <<std::endl;
	  m_infoStream << "lp:" << m_lastPartPose.x << " "  << m_lastPartPose.y << " "<< m_lastPartPose.theta <<std::endl;
	  m_infoStream << "op:" << m_odoPose.x << " " << m_odoPose.y << " "<< m_odoPose.theta <<std::endl;
	}
    }
	//std::cout<<"begin likelihoodAndScore:"<<time_diff.total_ms()<<std::endl;
    m_matcher.likelihoodAndScore(s, l, it->map, it->pose, plainReading);
	//std::cout<<"end likelihoodAndScore:"<<time_diff.total_ms()<<std::endl;
    sumScore+=score;
    it->weight+=l;
    it->weightSum+=l;

    //set up the selective copy of the active area
    //by detaching the areas that will be updated
	//std::cout<<"begin invalidateActiveArea:"<<time_diff.total_ms()<<std::endl;
    m_matcher.invalidateActiveArea();
	//std::cout<<"end invalidateActiveArea:"<<time_diff.total_ms()<<std::endl;

	//std::cout<<"begin computeActiveArea:"<<time_diff.total_ms()<<std::endl;
    m_matcher.computeActiveArea(it->map, it->pose, plainReading);
	//std::cout<<"end computeActiveArea:"<<time_diff.total_ms()<<std::endl;
  }
  if (m_infoStream)
    m_infoStream << "Average Scan Matching Score=" << sumScore/m_particles.size() << std::endl;	
}

inline void GridSlamProcessor::normalize(){
  //normalize the log m_weights
  double gain=1./(m_obsSigmaGain*m_particles.size());
  double lmax= -std::numeric_limits<double>::max();
  for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
    lmax=it->weight>lmax?it->weight:lmax;
  }
  //cout << "!!!!!!!!!!! maxwaight= "<< lmax << endl;
  
  m_weights.clear();
  double wcum=0;
  m_neff=0;
  for (std::vector<Particle>::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
    m_weights.push_back(exp(gain*(it->weight-lmax)));
    wcum+=m_weights.back();
    //cout << "l=" << it->weight<< endl;
  }
  
  m_neff=0;
  for (std::vector<double>::iterator it=m_weights.begin(); it!=m_weights.end(); it++){
    *it=*it/wcum;
    double w=*it;
    m_neff+=w*w;
  }
  m_neff=1./m_neff;
  
}

inline bool GridSlamProcessor::resample(const double* plainReading, int adaptSize, const RangeReading* reading){
  
  bool hasResampled = false;
  
  TNodeVector oldGeneration;
  for (unsigned int i=0; i<m_particles.size(); i++){
    oldGeneration.push_back(m_particles[i].node);
  }
  
  if (m_neff<m_resampleThreshold*m_particles.size()){		
    
    if (m_infoStream)
      m_infoStream  << "*************RESAMPLE***************" << std::endl;
    
    uniform_resampler<double, double> resampler;
    m_indexes=resampler.resampleIndexes(m_weights, adaptSize);
    
    if (m_outputStream.is_open()){
      m_outputStream << "RESAMPLE "<< m_indexes.size() << " ";
      for (std::vector<unsigned int>::const_iterator it=m_indexes.begin(); it!=m_indexes.end(); it++){
	m_outputStream << *it <<  " ";
      }
      m_outputStream << std::endl;
    }
    
    onResampleUpdate();
    //BEGIN: BUILDING TREE
    ParticleVector temp;
    unsigned int j=0;
    std::vector<unsigned int> deletedParticles;  		//this is for deleteing the particles which have been resampled away.
    
    //		cerr << "Existing Nodes:" ;
    for (unsigned int i=0; i<m_indexes.size(); i++){
      //			cerr << " " << m_indexes[i];
      while(j<m_indexes[i]){
	deletedParticles.push_back(j);
	j++;
			}
      if (j==m_indexes[i])
	j++;
      Particle & p=m_particles[m_indexes[i]];
      TNode* node=0;
      TNode* oldNode=oldGeneration[m_indexes[i]];
      //			cerr << i << "->" << m_indexes[i] << "B("<<oldNode->childs <<") ";
      node=new	TNode(p.pose, 0, oldNode, 0);
      //node->reading=0;
      node->reading=reading;
      //			cerr << "A("<<node->parent->childs <<") " <<endl;
      
      temp.push_back(p);
      temp.back().node=node;
      temp.back().previousIndex=m_indexes[i];
    }
    while(j<m_indexes.size()){
      deletedParticles.push_back(j);
      j++;
    }
    //		cerr << endl;
    std::cerr <<  "Deleting Nodes:";
    for (unsigned int i=0; i<deletedParticles.size(); i++){
      std::cerr <<" " << deletedParticles[i];
      delete m_particles[deletedParticles[i]].node;
      m_particles[deletedParticles[i]].node=0;
    }
    std::cerr  << " Done" <<std::endl;
    
    //END: BUILDING TREE
    std::cerr << "Deleting old particles..." ;
    m_particles.clear();
    std::cerr << "Done" << std::endl;
    std::cerr << "Copying Particles and  Registering  scans...";
    for (ParticleVector::iterator it=temp.begin(); it!=temp.end(); it++){
      it->setWeight(0);
      m_matcher.invalidateActiveArea();
      m_matcher.registerScan(it->map, it->pose, plainReading);
      m_particles.push_back(*it);
    }
    std::cerr  << " Done" <<std::endl;
    hasResampled = true;
  } else {
    int index=0;
    std::cerr << "Registering Scans:";
    TNodeVector::iterator node_it=oldGeneration.begin();
    for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++){
      //create a new node in the particle tree and add it to the old tree
      //BEGIN: BUILDING TREE  
      TNode* node=0;
      node=new TNode(it->pose, 0.0, *node_it, 0);
      
      //node->reading=0;
      node->reading=reading;
      it->node=node;

      //END: BUILDING TREE
      m_matcher.invalidateActiveArea();
      m_matcher.registerScan(it->map, it->pose, plainReading);
      it->previousIndex=index;
      index++;
      node_it++;
      
    }
    std::cerr  << "Done" <<std::endl;
    
  }
  //END: BUILDING TREE
  
  return hasResampled;
}
