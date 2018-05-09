/*
* Gene Foxwell 2018
*
* Reads in all image files from the given folder and runs the provided classification
* model on them.

Outputs a list of the top categories for each image in the folder.
*/

#include <sstream>
#include <iostream>
#include <string>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>

#include "imageNet.h"
#include "loadImage.h"
#include "cudaFont.h"

using namespace std;


/*
* Gets a vector of filenames from the target directory.
* taken from: https://www.linuxquestions.org/questions/programming-9/c-list-files-in-directory-379323/
*/
int getFileList(string dir, vector<string> &files) {
	
 	DIR *dp;
    	struct dirent *dirp;
	if((dp  = opendir(dir.c_str())) == NULL) {
		cout << "Error(" << errno << ") opening " << dir << endl;
		return errno;
	}

	while ((dirp = readdir(dp)) != NULL) {
		files.push_back(string(dirp->d_name));
	}
	
	closedir(dp);
	return 0;
}

int loadAndClassifyImage(string imgFilename, imageNet* net) {
	// load image from file on disk
	float* imgCPU    = NULL;
	float* imgCUDA   = NULL;
	int    imgWidth  = 0;
	int    imgHeight = 0;
		
	if( !loadImageRGBA(imgFilename.c_str(), (float4**)&imgCPU, (float4**)&imgCUDA, &imgWidth, &imgHeight) )
	{
		cout << "failed to load image: " << imgFilename << endl;
		return 0;
	}

	float confidence = 0.0f;
	
	// classify image
	const int img_class = net->Classify(imgCUDA, imgWidth, imgHeight, &confidence);

	// free resources
	CUDA(cudaFreeHost(imgCPU));

	return img_class;
}

int main(int argc, char** argv) {

	printf("imagenet-batch\n  args (%i):  ", argc);
	
	for( int i=0; i < argc; i++ )
		printf("%i [%s]  ", i, argv[i]);
		
	printf("\n\n");
	
	
	// retrieve directory location
	if( argc < 2 )
	{
		printf("imagenet-batch:   input directory required. \n");
		return 0;
	}
	
	string imgDirectory = argv[1];

	/*
	 * create imageNet
	 */
	imageNet* net = imageNet::Create(argc, argv);
	
	if( !net )
	{
		printf("imagenet-console:   failed to initialize imageNet\n");
		return 0;
	}

	/*
	* Get the list of files
	*/
	vector<string> fileList = vector<string>();
	getFileList(imgDirectory, fileList);

	/*
	* For each file in the list
	*	load the image file	
	* 	run the imagenet classifier on the file.
	*	output the resulting category.
	*/
	for(unsigned int i = 0; i < fileList.size(); i++) {
		if (fileList[i] == "." || fileList[i] == "..") {
			continue;		
		}

		string filename = imgDirectory + fileList[i];
		int result = loadAndClassifyImage(filename, net);		
		cout << "Image: " << filename << " classification: " << net->GetClassDesc(result) << endl;
	}

	delete net;

	return 0;

}

