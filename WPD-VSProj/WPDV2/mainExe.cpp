#include "PCSimplification.h"

int main(int argc, char* argv[])
{
	
	cout << "start!" << endl;
	char* fileSourceChar = NULL;
	char* fileTargetChar = NULL;
	int simNum; 	
	
	if (argc == 4) {
		fileSourceChar = argv[1];
		fileTargetChar = argv[2];
		simNum = atoi(argv[3]);		
	}
		
	string fileBase(fileSourceChar);
	string saveBase(fileTargetChar);	

	PCSimplification PCSim(fileBase, saveBase);
	PCSim.SetResultNumber(simNum);
	PCSim.SetVoronoi(true);
	PCSim.SetVoronoiNumber(5);	
	PCSim.doSimplification();

	return 0;

}



