#include "utils.h"


utils::utils(void)
{
	config = NULL;
}


utils::~utils(void)
{
}

void utils::initScanLidar(xmlConfig * _config)
{
	config = _config;

	try
	{
		_OriginalMap = cvLoadImage(config->map.mapFile.c_str());
		IplImage image = MAPS::IplImageModel(_OriginalMap->width, _OriginalMap->height, MAPS_CHANNELSEQ_RGB);
		
		_bIsMapLoaded = true;

		_dXmin = -config->map.mapPixelCalib.pixelX0 * config->map.mapPixelCalib.pixelSize;
		_dXmax = (_OriginalMap->width - config->map.mapPixelCalib.pixelX0) * config->map.mapPixelCalib.pixelSize ;
		_dYmin = -config->map.mapPixelCalib.pixelY0 * config->map.mapPixelCalib.pixelSize;
		_dYmax = (_OriginalMap->height - config->map.mapPixelCalib.pixelY0) * config->map.mapPixelCalib.pixelSize;


	}
	catch(cv::Exception e)
	{
		MAPSString s ("cv::Exception thrown: ");
		s+=e.what(); //Est cens?fournir un message explicite sur le pourquoi du comment de l’exception.
		MAPS::ReportError(s);
		_bIsMapLoaded = false;
	}
}



double utils::rayTracing(int xPix, int yPix, double Alpha)
{
	
	CvPoint lidar_source;
    lidar_source.x = xPix;
    lidar_source.y = yPix;//_OriginalMap->height - yPix;

    // Calcul de l'extrémit?du rayon
    CvPoint extremite;
	extremite.x = lidar_source.x + (config->lidar.ray.distanceMax/config->map.mapPixelCalib.pixelSize)*cos(Alpha); 
    extremite.y = lidar_source.y + (config->lidar.ray.distanceMax/config->map.mapPixelCalib.pixelSize)*(-sin(Alpha));
	
	// Récupération des pixels le long du rayon


    LineIterator iterator(_OriginalMap, lidar_source, extremite, 4);

    double distance = config->lidar.ray.distanceMax / config->map.mapPixelCalib.pixelSize;

    for(int i = 0; i < iterator.count; i++)
    {
		//CvPoint pt = iterator.pos();
		
		if(! (*iterator.ptr ))
		{
			CvPoint pt = iterator.pos();
			distance = sqrt((double)((pt.x - lidar_source.x)*(pt.x - lidar_source.x)+(pt.y - lidar_source.y)*(pt.y - lidar_source.y))); 
			return(distance);
		}


	/*	if(!isMapFree(pt.x, pt.y))
		{
			distance = sqrt((double)((pt.x - lidar_source.x)*(pt.x - lidar_source.x)+(pt.y - lidar_source.y)*(pt.y - lidar_source.y))); 
			return(distance);
			break;	// quitte la boucle en live !!!
		}*/
		iterator++;

		

    }
	return(distance);
}


void utils::lidarSim(double pose[3], vector<double> &Distances)
{
	if(config==NULL)
		return;
	
	double angleLidar = config->lidar.Hfov.startAngle ;
	// todo tenir compte du déplacement du robot qui s'ajoute ?rotation du lidar
	double xPix,yPix;
	positionToMap(pose[0],  pose[1], xPix, yPix);

	Distances.clear();
	
	while(angleLidar <= config->lidar.Hfov.endAngle)
	{
		double distance = rayTracing(xPix, yPix, pose[2] + (angleLidar)*M_PI/180.0) * config->map.mapPixelCalib.pixelSize ;
		if( distance < config->lidar.ray.distanceMax  && distance>config->lidar.ray.distanceMin)
		{
			
		}
		else
		{
			distance = config->lidar.ray.noEchoValue;
		}
		Distances.push_back(distance);
		angleLidar += config->lidar.Hfov.resolution;
	}

}

void utils::positionToMap(double x, double y, double &xPix, double &yPix)
{
	xPix = x / config->map.mapPixelCalib.pixelSize + config->map.mapPixelCalib.pixelX0 ;
	yPix = _OriginalMap->height - (y / config->map.mapPixelCalib.pixelSize + config->map.mapPixelCalib.pixelY0);
}