/* EE201A Winter 2017 Course Project
 * Author: Vishesh Dokania (vdokania@ucla.edu)
 */
// v3.3

#include <iostream>
#include "oaDesignDB.h"
#include <vector>
#include "InputOutputHandler.h"
#include "ProjectInputRules.h"
#include "OAHelper.h"


//costmized header file
#include <fstream>
#include <set>
#include <string.h>
#include <iomanip>
#include <stdlib.h>
#include <limits.h>
#include <math.h>

using namespace oa;
using namespace std;

static oaNativeNS ns;
/*
 * 
 */
struct side 
{
	std::set <oa::oaCoord> list;
	std::set <oa::oaCoord> * pNext;
	std::set <oa::oaCoord> * pPrevious;
	side(){};

};
struct macro
{	
	macro()
	{
		this->lside.pNext = &this->tside.list;
		this->tside.pNext = &this->rside.list;
		this->rside.pNext = &this->bside.list;
		this->bside.pNext = &this->lside.list;

		this->lside.pPrevious = &this->bside.list;
		this->bside.pPrevious = &this->rside.list;
		this->rside.pPrevious = &this->tside.list;
		this->tside.pPrevious = &this->lside.list;
		this->isCounted = 0;
	};
	oaString name;
	oaInst * pU;
	oaCoord t;
	oaCoord b;
	oaCoord l;
	oaCoord r;
	oaPoint o;
	side lside;
	side rside;
	side tside;
	side bside;
	oaCoord l_legal;
	oaCoord r_legal;
	oaCoord t_legal;
	oaCoord b_legal;
	oaCoord xStart;
	oaCoord yStart;
	
	bool isCounted;
	void reset_isCounted(){
		this->isCounted =0;
	};
	~macro(){};
};
struct net 
{
	int num_of_instTerm;
	int num_of_term;
	int sigType;
	oaString netName;
	oaInstTerm **   instTermP;
	oaTerm ** termP;
	oaInstTerm **  reset_IT;
	oaTerm ** reset_T;

	oaPoint * initList;
	oaPoint instBuoy;

	void reset_instTermP(){
		this->instTermP = this->reset_IT;
	};
	void reset_termP(){
		this->termP = this->reset_T;
	};
	void reset_all(){
		this->termP = this->reset_T;
		this->instTermP = this->reset_IT;
	};
	void save_initial_pointer(oaInstTerm ** it, oaTerm ** t){

		this->reset_T = t;
		this->reset_IT = it;
	};	;
	~net(){};
};

static int size_macro_array =0;
oaDist pinMoveStep;
macro * pMacroArray;
int pertubation;
int range;	// Ratio of pin pitch and move step


int main(int argc, char *argv[])
{	
    //Hello World
	cout << "=================================================" << endl;
	cout << "Automated Inter-Chip Pin Assignment" << endl;
	cout << "UCLA EE 201A Winter 2017 Course Project" << endl;
	cout << endl;
	cout << "<TEAM NUMBER: 6>" << endl;
	cout << "<TEAM MEMBERS: Junhao Hua & Silei Ma>" << endl;
	cout << "<STUDENT IDS: 304734647 & 604741646>" << endl;
	cout << "=================================================" << endl << endl;

    //Usage
	cout << "Ensure you have an existing OA design database before running this tool. Also please adhere to the following command-line usage:" << endl;
	cout << "./PinAssign <DESIGN NAME> <OUTPUT DESIGN NAME> <INPUT RULE FILE NAME> <MACRO INFO FILENAME>" << endl;
	cout << "For example:" << endl;            
	cout << "./PinAssign sbox_x2 sbox_x2_minrule benchmarks/bench1/min.inputrules logs/pinassign_sbox_x2_minrule.macros" << endl;

	// Initialize OA with data model 3
	oaDesignInit(oacAPIMajorRevNumber, oacAPIMinorRevNumber, 3);
	oaRegionQuery::init("oaRQSystem");

    //Read in design library
	cout << "\nReading design library..." << endl;
	DesignInfo designInfo;
	InputOutputHandler::ReadInputArguments(argv, designInfo);
	oaLib* lib;
	oaDesign* design= InputOutputHandler::ReadOADesign(designInfo, lib);

	// Get the TopBlock for this design.
	oaBlock* block= InputOutputHandler::ReadTopBlock(design);

	// Fetch all instances in top block and save a unique master design copy for each
	cout << "\nSaving copies of each unique macro instance..." << endl;
	InputOutputHandler::SaveMacroDesignCopies(designInfo, block);
	
    // now, get the input rules from file
	cout << "\nReading input rules..." << endl;
	ProjectInputRules inputRules(designInfo.inputRuleFileName); 
	inputRules.print();		

	cout << "\nBeginning pin assignment..." << endl;
	//=====================================================================
    // All pin assignment code should be handled here
	// The scratch code below covers basic traversal and some useful functions provided
	// You are free to edit everything in this section (marked by ==)
	oaString netName, instName, masterCellName, assocTermName, termName;
	oaIter<oaNet> netIterator(block->getNets());
	
	// Functions Declaration
	macro * onWhichOne(macro * pMA, oaInstTerm * pIT);
	oaPoint getIT_Pos(oaInstTerm * ptr);
	oaPoint getT_Pos(oaTerm * ptr);
	oaPoint getLL(oaInst * ptr);
	oaPoint getUR(oaInst * ptr);
	oaPoint getStart(oaInst * ptr);
	int isOccupied(macro * tempMacro, oaPoint* Aim, bool isOverPert);
	int isNearTheCorner(oaPoint* pPoint, macro * pM,side * pSide,int side,bool clkwise);
	void getPort(oaPoint Buoy, macro aMacro, oaPoint *Port);
	void getAim(oaInstTerm *instTermP , oaPoint *Port, macro *pMacroArray, oaPoint *Aim, oaPoint *LegalAim, bool *clkwise, int pertu);
	void getInstBuoy(net Net, macro *pMacroArray, oaPoint *InstBuoy);
	oaCoord getUpRightLegal(oaCoord illegal, oaCoord ref);
	oaCoord getDownLeftLegal(oaCoord illegal, oaCoord ref);
	int calManhattan(oaPoint point_1, oaPoint point_2);
	void moveAlongSide(oaPoint * LegalAim, int side ,bool clkwise);
	void eraseAim(macro *tempMacro, oaPoint* Aim);
	int MoveLegalAim(oaPoint *LegalAim, macro *tempMacro, int currentSide, bool isOverPert, bool clkwise);
	
	// Construct new database using map
	int size_net = block->getNets().getCount();
	size_macro_array = (block -> getInsts()).getCount();
	net * pNet = new net[size_net];
	
	// Get max pertubation
	if(inputRules.getMaxPinPerturbation()!=-1)
		pertubation = 2000* inputRules.getMaxPinPerturbation();
	else pertubation = INT_MAX;
	
	// Get move step and pitch
	pinMoveStep = oaDist(2000*inputRules.getPinMoveStep());
	range  = 2000*inputRules.getMinPinPitch()/pinMoveStep;
	
	//cout<< "\nPin Layer : "<<inputRules.getPinLayer()<<endl;
	//cout<< "Minimum Move Step : "<< pinMoveStep << endl;
	//cout<< "Minimum Pin Pitch : "<< range * pinMoveStep << endl;
	//cout<< "Maximum PinPertubation : "<< pertubation << endl;
		
	//save the starting point
	pMacroArray = new macro[size_macro_array];

	// get Macro
	int id = 0;
	oaIter<oaInst> instIT(block -> getInsts());
	while (oaInst * inst = instIT.getNext())
	{	
		pMacroArray[id].pU  = inst;
		oaString name;
		inst->getName(ns,name);
		pMacroArray[id].name = name;
		oaBox tempBox;
		inst->getBBox(tempBox);
		pMacroArray[id].l =getLL(inst).x();
		pMacroArray[id].b =getLL(inst).y();
		pMacroArray[id].r =getUR(inst).x();
		pMacroArray[id].t =getUR(inst).y();
		pMacroArray[id].xStart=getStart(inst).x();
		pMacroArray[id].yStart=getStart(inst).y();
		pMacroArray[id].l_legal=getUpRightLegal(pMacroArray[id].l, pMacroArray[id].xStart) + ceil(0.5*range)*pinMoveStep;
		pMacroArray[id].r_legal=getDownLeftLegal(pMacroArray[id].r, pMacroArray[id].xStart) - ceil(0.5*range)*pinMoveStep;
		pMacroArray[id].b_legal=getUpRightLegal(pMacroArray[id].b, pMacroArray[id].yStart) + ceil(0.5*range)*pinMoveStep;
		pMacroArray[id].t_legal=getDownLeftLegal(pMacroArray[id].t, pMacroArray[id].yStart) - ceil(0.5*range)*pinMoveStep;		
		//cout<< "Reference Point (" << pMacroArray[id].xStart << ", " << pMacroArray[id].yStart << ")" << endl;
		//cout<<" Left "<<pMacroArray[id].l<<" ---> "<<pMacroArray[id].l_legal<<"         "<<pMacroArray[id].r_legal<<" <--- "<<pMacroArray[id].r<<" Right "<<endl;
		//cout<<" Bottom "<<pMacroArray[id].b<<" ---> "<<pMacroArray[id].b_legal<<"         "<<pMacroArray[id].t_legal<<" <--- "<<pMacroArray[id].t<<" Top "<<endl;		
		id ++;
	}		
	//cout<<std::left;
	//cout << "Total # of Macro: "<<setw(18) << std::right<<size_macro_array<<endl;
	//cout << "Total # of Nets: " <<setw(20) << std::right<<size_net<<endl;
	
	// Set net counter
	int count_net = 0;
	while (oaNet* net = netIterator.getNext()) {
		net->getName(ns, netName);
		int size_instTerm = net->getInstTerms().getCount();
		int size_term = net-> getTerms().getCount();
		pNet[count_net].netName = netName; 
		pNet[count_net].sigType = net->getSigType(); 
		// initiating the array
		pNet[count_net].instTermP = (oaInstTerm **)malloc(sizeof(oaInstTerm *) * (size_instTerm));
		pNet[count_net].termP = (oaTerm **)malloc(sizeof(oaTerm *) *(1+size_term));	
		pNet[count_net].save_initial_pointer(pNet[count_net].instTermP, pNet[count_net].termP );
		pNet[count_net].num_of_instTerm = size_instTerm;
		// initiating the initials
		pNet[count_net].initList = new oaPoint[size_instTerm];
		pNet[count_net].num_of_term = size_term;
		oaIter<oaInstTerm> instTermIterator(net->getInstTerms());
		oaIter<oaTerm> termIterator(net->getTerms());
		int id = 0 ;
		while (oaInstTerm* instTerm = instTermIterator.getNext()) {
			instTerm->getTermName(ns, assocTermName);
			oaPoint instTermPos = OAHelper::GetAbsoluteInstTermPosition(instTerm);
			oaString netName;
			oaInst* inst = instTerm->getInst();
			inst->getName(ns, instName);
			inst->getCellName(ns, masterCellName);
			instTerm ->getNet(false)->getName(ns,netName); 
			*pNet[count_net].instTermP = instTerm;
			pNet[count_net].initList[id] = getIT_Pos(instTerm);
			id++;
			pNet[count_net].instTermP++;
			}
		//Terms
		while (oaTerm* term = termIterator.getNext()) {
			term->getName(ns,termName);
			oaPoint termPos = OAHelper::GetTermPosition(term);
			*pNet[count_net].termP= term;
			pNet[count_net].termP++;
		}
		count_net++;
	}
	
	
	// Begin Assignment
	for (int i = 0; i < size_net; ++i)
    {	
		// Filter the floating nets.
		// Nets contains Terms
		if(( pNet[i].sigType == 0 ) && ( pNet[i].num_of_term != 0 ))
		{
			pNet[i].reset_all();
			
			// Save initial positions
			oaPoint *initial= new oaPoint[pNet[i].num_of_instTerm];
			pNet[i].reset_instTermP();
			for (int j = 0; j < pNet[i].num_of_instTerm; ++j)
			{
			*initial =	getIT_Pos(*(pNet[i].instTermP));
			pNet[i].instTermP++;
			initial++;
			}
			initial=initial-pNet[i].num_of_instTerm;
			oaPoint *reset;
			reset=initial;
			
			for (int j = 0 ; j < pNet[i].num_of_term;++j)
			{ 	
				initial=reset;
				pNet[i].reset_instTermP();
				for (int k = 0; k < pNet[i].num_of_instTerm; ++k)
				{	
					oaPoint *Port = new oaPoint();
					oaPoint *Aim = new oaPoint();
					oaPoint *LegalAim = new oaPoint();
					
					// For deep improvement(it will significantly slow the process)
					oaPoint *LegalAim_b = new oaPoint();
					oaPoint *LegalAim_d = new oaPoint();
					oaPoint *LegalAim_ori = new oaPoint();
					
					bool * clkwise = new bool();
					macro * tempMacro = onWhichOne(pMacroArray,*(pNet[i].instTermP));
					// Find Legal Aim
					// Term acts as Buoy for all InstTerms
					getPort(getT_Pos(*(pNet[i].termP)), *tempMacro, Port);
					getAim(*(pNet[i].instTermP), Port, pMacroArray, Aim, LegalAim, clkwise, pertubation);
					bool cheated=false;
					
					// Check spacing and move	
					// Duplicate LegalAim for comparing move directions
					*LegalAim_b = *LegalAim;
					*LegalAim_d = *LegalAim;
					*LegalAim_ori = *LegalAim;
					
					// Manhattan distance between Initial point and LegalAim
					int distance=0, distance_b=0;
					bool isOverPert=(distance>pertubation);				
					int currentSide = isOccupied(tempMacro, LegalAim, isOverPert);
					
					// For deep improvement(it will significantly slow the process)
					bool isOverPert_b=(distance_b>pertubation);
					int currentSide_b = isOccupied(tempMacro, LegalAim_b, isOverPert_b);
					int currentSide_d = isOccupied(tempMacro, LegalAim_d, 0);
					// Two dead condition
					bool dead_clk=false, dead_cclk=false;
					
					// Move if the position is occuipied by others or leading to pertubation violation
					// Clockwise Move	
					while ((currentSide !=0)||isOverPert){						
						distance=calManhattan(*initial, *LegalAim);
						isOverPert=(distance>pertubation);
						currentSide=MoveLegalAim(LegalAim, tempMacro, currentSide, isOverPert, *clkwise);
						// Check pertubation again after moving LegalAim
						distance=calManhattan(*initial, *LegalAim);
						isOverPert=(distance>pertubation);
						if(currentSide==0&&isOverPert) eraseAim(tempMacro, LegalAim);
						// Test Error
						if(*LegalAim==*LegalAim_ori){
							cout<<"Larger Spaceing needed for "<<pNet[i].netName<<" in clockwise(contains Terms) "<<endl;
							dead_clk=true;
							*LegalAim=*LegalAim_b;
							break;
						}
					}
					
					// Counter-Clockwise Move
					while ((currentSide_b !=0)||isOverPert_b){
						distance_b=calManhattan(*initial, *LegalAim_b);
						isOverPert_b=(distance_b>pertubation);
						currentSide_b=MoveLegalAim(LegalAim_b, tempMacro, currentSide_b, isOverPert_b, !*clkwise);
						// Check pertubation again after moving LegalAim_b
						distance_b=calManhattan(*initial, *LegalAim_b);
						isOverPert_b=(distance_b>pertubation);
						if(currentSide_b==0&&isOverPert_b) eraseAim(tempMacro, LegalAim_b);
						// Test Error
						if(*LegalAim_b==*LegalAim_ori){
							cout<<"Larger Spaceing needed for "<<pNet[i].netName<<" in counter-clockwise(contains Terms) "<<endl;
							dead_cclk=true;
							*LegalAim_b=*LegalAim;
							break;
						}
					}
					
					// Here's cheating: Move without consideration of pertubation
					if(dead_clk&&dead_cclk){
						while ((currentSide_d !=0)){						
						currentSide_d=MoveLegalAim(LegalAim_d, tempMacro, currentSide_d, 0, !*clkwise);
						}
						*LegalAim=*LegalAim_d;
						cheated=true;
						cout<<"Move to pseudo target!!"<<endl;
					}
					
					// If can not find available position in clockwise
					else if(dead_clk)
						*LegalAim=*LegalAim_b;
					
					// If can find available positions in both direction
					else{
						// Choose the better direction while erase the other
						if(calManhattan(*LegalAim_ori, *LegalAim)>calManhattan(*LegalAim_ori, *LegalAim_b)){
							eraseAim(tempMacro, LegalAim);
							*LegalAim=*LegalAim_b;
						}
						else if(*LegalAim!=*LegalAim_b)
							eraseAim(tempMacro, LegalAim_b);	
					}								
					OAHelper::MovePinToPosition(*(pNet[i].instTermP), *LegalAim);
					pNet[i].instTermP++;
					initial++;
				}
				pNet[i].termP++;
			}
		}
		
		// Nets contain only instTerms
		else if (( pNet[i].sigType == 0 ) && ( pNet[i].num_of_term == 0 )){
			oaInstTerm ** temp1 = pNet[i].reset_IT;	
			// Save initial positions
			oaPoint *initial= new oaPoint[pNet[i].num_of_instTerm];
			pNet[i].reset_instTermP();
			for (int j = 0; j < pNet[i].num_of_instTerm; ++j)
			{
			*initial =	getIT_Pos(*(pNet[i].instTermP));
			pNet[i].instTermP++;
			initial++;
			}
			initial=initial-pNet[i].num_of_instTerm;
			oaPoint *reset;
			reset=initial;
			
			// Iterate all InstTerms and make each InstTerm as buoy for others
			for (int k = 0; k < pNet[i].num_of_instTerm; ++k)
			{
				initial=reset;
				pNet[i].reset_instTermP();
				for (int j = 0; j < pNet[i].num_of_instTerm; ++j)
				{	 
					oaPoint *Port = new oaPoint();
					oaPoint *Aim = new oaPoint();
					oaPoint *LegalAim = new oaPoint();
					
					// For deep improvement(it will significantly slow the process)
					oaPoint *LegalAim_b = new oaPoint();
					oaPoint *LegalAim_d = new oaPoint();
					oaPoint *LegalAim_db = new oaPoint();
					oaPoint *LegalAim_ori = new oaPoint();
					
					bool * clkwise = new bool();
					macro * tempMacro = onWhichOne(pMacroArray,*(pNet[i].instTermP));
					getPort(getIT_Pos(*temp1), *tempMacro, Port);
					// Pertubation for each stages of (# of InstTerms - 1) stages
					int newpertubation = pertubation/(pNet[i].num_of_instTerm - 1);
					getAim(*(pNet[i].instTermP), Port, pMacroArray, Aim, LegalAim, clkwise, newpertubation);
					bool cheated=false;
					
					// If it's the last stage, check spacing and move
					if(k==pNet[i].num_of_instTerm-1)
					{							
						// Manhattan distance between Initial point and LegalAim
						int distance=0, distance_b=0;
						bool isOverPert=(distance>pertubation);
						int currentSide = isOccupied(tempMacro, LegalAim, isOverPert);
						
						// For deep improvement(it will significantly slow the process)
						// Duplicate LegalAim for comparing move directions
						*LegalAim_b = *LegalAim;
						*LegalAim_d = *LegalAim;
						*LegalAim_db = *LegalAim;
						*LegalAim_ori = *LegalAim;
						bool isOverPert_b=(distance_b>pertubation);
						int currentSide_b = isOccupied(tempMacro, LegalAim_b, isOverPert_b);
						int currentSide_d = isOccupied(tempMacro, LegalAim_d, 0);
						int currentSide_db = isOccupied(tempMacro, LegalAim_d, 0);
						// Two dead condition
						bool dead_clk=false, dead_cclk=false;
						
						// Move if the position is occuipied by others or leading to pertubation violation
						// Clockwise Move	
						while ((currentSide !=0)||isOverPert){						
							distance=calManhattan(*initial, *LegalAim);
							isOverPert=(distance>pertubation);
							currentSide=MoveLegalAim(LegalAim, tempMacro, currentSide, isOverPert, *clkwise);
							// Check pertubation again after moving LegalAim
							distance=calManhattan(*initial, *LegalAim);
							isOverPert=(distance>pertubation);
							if(currentSide==0&&isOverPert) eraseAim(tempMacro, LegalAim);
							// Test Error
							if(*LegalAim==*LegalAim_ori){
								cout<<"Larger Spaceing needed for "<<pNet[i].netName<<" in clockwise "<<endl;
								*LegalAim=*LegalAim_b;
								dead_clk=true;
								break;
							}
						}
						
						// Counter-Clockwise Move
						while ((currentSide_b !=0)||isOverPert_b){
							distance_b=calManhattan(*initial, *LegalAim_b);
							isOverPert_b=(distance_b>pertubation);
							currentSide_b=MoveLegalAim(LegalAim_b, tempMacro, currentSide_b, isOverPert_b, !*clkwise);	
							// Check pertubation again after moving LegalAim_b
							distance_b=calManhattan(*initial, *LegalAim_b);
							isOverPert_b=(distance_b>pertubation);
							if(currentSide_b==0&&isOverPert_b) eraseAim(tempMacro, LegalAim_b);
							// Test Error
							if(*LegalAim_b==*LegalAim_ori){
								cout<<"Larger Spaceing needed for "<<pNet[i].netName<<" in counter-clockwise "<<endl;
								*LegalAim_b=*LegalAim;
								dead_cclk=true;
								break;
							}
						}
						
						// Here's cheating: Move without consideration of pertubation
						if(dead_clk&&dead_cclk){
							// Clockwise 
							while ((currentSide_d !=0)){						
							currentSide_d=MoveLegalAim(LegalAim_d, tempMacro, currentSide_d, 0, *clkwise);
							}
							// Counter-Clockwise
							while((currentSide_db !=0)){						
							currentSide_db=MoveLegalAim(LegalAim_db, tempMacro, currentSide_db, 0, !*clkwise);
							}
							// Choose the closer
							if(calManhattan(*LegalAim_ori, *LegalAim_d)>calManhattan(*LegalAim_ori, *LegalAim_db)){
								eraseAim(tempMacro, LegalAim_d);
								*LegalAim_d=*LegalAim_db;
							}
							else if(*LegalAim_d!=*LegalAim_db)
								eraseAim(tempMacro, LegalAim_db);
							*LegalAim=*LegalAim_d;
							cheated=true;
							cout<<"Move to pseudo target!!"<<endl;
						}
						
						// If can not find available position in clockwise
						else if(dead_clk)
							*LegalAim=*LegalAim_b;
						
						// If can find available positions in both direction
						else{
							// Choose the better direction while erase the other
							if(calManhattan(*LegalAim_ori, *LegalAim)>calManhattan(*LegalAim_ori, *LegalAim_b)){
								eraseAim(tempMacro, LegalAim);
								*LegalAim=*LegalAim_b;
							}
							else if(*LegalAim!=*LegalAim_b)
								eraseAim(tempMacro, LegalAim_b);	
						}
					}					
					OAHelper::MovePinToPosition(*(pNet[i].instTermP), *LegalAim);
					if(cheated){
						oaInstTerm** InstSave;
						InstSave=pNet[i].instTermP;
						bool swap1=false;
						bool swap2=false;
						cout<<"Let's swap"<<endl;
						// Swap 1 time
						for(int l =0 ; l<i;++l){
							if(pNet[l].sigType == 0){
								pNet[l].reset_instTermP();
								for(int n=0; n < pNet[l].num_of_instTerm; ++n){
									// *****
									macro * thatMacro = onWhichOne(pMacroArray,*(pNet[l].instTermP));
									// Substitute is close to target
									bool condition1=(calManhattan(pNet[l].initList[n], *LegalAim_d)<pertubation);
									// And close to victim's initial position
									bool condition2=(calManhattan(*initial, getIT_Pos(*pNet[l].instTermP))<pertubation);
									// They are on same macro
									bool condition3=(thatMacro==tempMacro);
									if(condition1&&condition2&&condition3){
									cout<<pNet[i].netName<<" swapped with : "<< pNet[l].netName <<endl;
									OAHelper::MovePinToPosition(*(pNet[i].instTermP), getIT_Pos(*(pNet[l].instTermP)));
									cout<<"Which is : "<< pNet[l].netName <<endl;
									OAHelper::MovePinToPosition(*(pNet[l].instTermP), *LegalAim);
									swap1=true;
									goto door;
									}
								pNet[l].instTermP++;
								}
							}
						}

						if(!swap1){
							cout<<"Need 2 times swap"<<endl;
							for(int l =0 ; l<i;++l){
								if(pNet[l].sigType == 0){
									pNet[l].reset_instTermP();
									for(int n=0; n < pNet[l].num_of_instTerm; ++n){
										macro * thatMacro = onWhichOne(pMacroArray,*(pNet[l].instTermP));
										// Close to target
										bool condition1=(calManhattan(pNet[l].initList[n], *LegalAim_d)<pertubation);
										// Not too far from initial
										bool condition2=(calManhattan(getIT_Pos(*pNet[l].instTermP), *initial)<1.5*pertubation);
										// Same macro
										bool condition3=(thatMacro==tempMacro);
										if(condition1&&condition2&&condition3){
										cout<<pNet[i].netName<<" firstly swapped with : "<< pNet[l].netName <<endl;
										OAHelper::MovePinToPosition(*(pNet[i].instTermP), getIT_Pos(*(pNet[l].instTermP)));
										cout<<"Which is : "<< pNet[l].netName <<endl;
										OAHelper::MovePinToPosition(*(pNet[l].instTermP), *LegalAim);
										goto second;
										}
									pNet[l].instTermP++;
									}
								}
							}
							second:
							for(int l =0 ; l<i;++l){
								if(pNet[l].sigType == 0){
									pNet[l].reset_instTermP();
									for(int n=0; n < pNet[l].num_of_instTerm; ++n){
										macro * thatMacro = onWhichOne(pMacroArray,*(pNet[l].instTermP));
										// Close to first swapped position
										bool condition1=(calManhattan(pNet[l].initList[n], getIT_Pos(*(pNet[i].instTermP)))<pertubation);
										// Close to initial
										bool condition2=(calManhattan(*initial, getIT_Pos(*pNet[l].instTermP))<pertubation);
										// Same macro
										bool condition3=(thatMacro==tempMacro);
										if(condition1&&condition2&&condition3){
										cout<<pNet[i].netName<<" secondly swapped with : "<< pNet[l].netName <<endl;
										oaPoint temp = getIT_Pos(*(pNet[i].instTermP));
										OAHelper::MovePinToPosition(*(pNet[i].instTermP), getIT_Pos(*(pNet[l].instTermP)));
										cout<<"Which is : "<< pNet[l].netName <<endl;
										OAHelper::MovePinToPosition(*(pNet[l].instTermP), temp);
										swap2=true;
										goto door;
										}
									pNet[l].instTermP++;
									}
								}
							}
						}
						door:
						cout<<endl;
						pNet[i].instTermP=InstSave;
					}
					pNet[i].instTermP++;
					initial++;
				}
			temp1++;
			}	
		}
	}
	
	
	// Output final positions
	/*int ount=0;
	cout<< "\nFinal Positions "<<setw(20)<<" Initial Positions "<<endl; 
	for (int i = 0; i < size_net; ++i)
	{	
        pNet[i].reset_all();
		for (int j = 0; j < pNet[i].num_of_instTerm; ++j)
		{
			oaPoint foo = getIT_Pos(*(pNet[i].instTermP));
			oaPoint fooo = pNet[i].initList[j];
			cout << pNet[i].netName<< "\t"<< foo.x() <<"\t"<< foo.y()<< "\t"<< fooo.x() <<"\t"<< fooo.y()<<endl;
			pNet[i].instTermP++;
			if (pNet[i].sigType == 0)
			ount++;
		}
	}
	cout<<"\n"<<ount<<" pins!!"<<endl;*/
	
	
	// Output list
	/*int count=0;
	cout<< "\nList "<<endl;
	for(int id=0; id<size_macro_array;id++){
		cout<<"\n\nMarco "<<id<<" :\n"<<endl;
		cout<<"Left Side :"<<endl;
		for(std::set<oaCoord>::iterator it = pMacroArray[id].lside.list.begin(); it != pMacroArray[id].lside.list.end() ; it++){
			cout <<pMacroArray[id].l<<", "<<  *it<<endl;
			count++;
		}
		cout<<"\nRight Side :"<<endl;
		for(std::set<oaCoord>::iterator it = pMacroArray[id].rside.list.begin(); it != pMacroArray[id].rside.list.end() ; it++){
			cout <<pMacroArray[id].r<<", "<<  *it<<endl;
			count++;
		}
		cout<<"\nTop Side :"<<endl;
		for(std::set<oaCoord>::iterator it = pMacroArray[id].tside.list.begin(); it != pMacroArray[id].tside.list.end() ; it++){
			cout <<  *it<<", "<<pMacroArray[id].t<<endl;
			count++;
		}
		cout<<"\nBottom Side :"<<endl;
		for(std::set<oaCoord>::iterator it = pMacroArray[id].bside.list.begin(); it != pMacroArray[id].bside.list.end() ; it++){
			cout <<  *it<<", "<<pMacroArray[id].b<<endl;
			count++;
		}
	}
	cout<<"\n"<<count<<" in the list!!"<<endl;*/
	
	
    //Save the improved version of the design
	InputOutputHandler::SaveAndCloseAllDesigns(designInfo, design, block);
	
	if (lib)
		lib->close();

	cout << endl << "\nDone!" << endl;
	return 0;
}
	
	
// Functions	
macro *onWhichOne(macro * pMA, oaInstTerm * pIT){
	oaString name;
	pIT -> getInst()->getName(ns,name);

	for (int i = 0; i < size_macro_array; ++i)
	{	
		if (name == pMA[i].name)
		{		
			return & pMA[i];
		}
	}
};


oaPoint getIT_Pos(oaInstTerm * ptr){
	oaPoint center;
	center = OAHelper::GetAbsoluteInstTermPosition(ptr);
	return center;
};


oaPoint getT_Pos(oaTerm * ptr){
	oaPoint center;
	oaIter<oaPin>  pinIT(ptr->getPins());
	oaIter<oaPinFig> pinfigIT(pinIT.getNext()->getFigs());
	oaBox box;
	pinfigIT.getNext()->getBBox(box);
	box.getCenter(center);
	return center;
};


oaPoint getLL(oaInst * ptr){
	int index = 0;
	oaPoint min;
	oaIter<oaInstTerm> instTIT(ptr ->getInstTerms());
	while (oaInstTerm * pit = instTIT.getNext()){
		if (index ==0)
		{
			min = OAHelper::GetAbsoluteInstTermPosition(pit);
			index++;
		}else{
			oaPoint temp=OAHelper::GetAbsoluteInstTermPosition(pit);
			if (min.x() >  temp.x())
			{
				min.x() = temp.x();
			}
			if (min.y() > temp.y())
			{
				min.y() = temp.y();
			}
		}
	}
	return min;
};


oaPoint getStart(oaInst * ptr){
	int index = 0;
	oaPoint Start;
	Start.x()=0;
	Start.y()=0;
	oaPoint min;
	oaPoint max;
	oaIter<oaInstTerm> instTIT(ptr ->getInstTerms());
	while (oaInstTerm * pit = instTIT.getNext()){
		if (index ==0)
		{
			min = OAHelper::GetAbsoluteInstTermPosition(pit);
			index++;
		}else{
			oaPoint temp=OAHelper::GetAbsoluteInstTermPosition(pit);
			if (min.x() >  temp.x())
			{
				min.x() = temp.x();
			}
			if (min.y() > temp.y())
			{
				min.y() = temp.y();
			}
		}
	}
	oaIter<oaInstTerm> instTIT2(ptr ->getInstTerms());
	while (oaInstTerm * pit2 = instTIT2.getNext()){
		oaPoint temp=OAHelper::GetAbsoluteInstTermPosition(pit2);
		if (min.x() ==  temp.x())
		{
			Start.y() = temp.y();
		}
		if (min.y() == temp.y())
		{
			Start.x() = temp.x();
		}
	}
	return Start;
};	
	

oaPoint getUR(oaInst * ptr){
	int index = 0;
	oaPoint max;
	oaIter<oaInstTerm> instTIT(ptr ->getInstTerms());
	while (oaInstTerm * pit = instTIT.getNext()){
		if (index ==0)
		{
			max = OAHelper::GetAbsoluteInstTermPosition(pit);
			index++;
		}else{
			oaPoint temp=OAHelper::GetAbsoluteInstTermPosition(pit);
			if (max.x() <  temp.x())
			{
				max.x() = temp.x();
			}
			if (max.y() < temp.y())
			{
				max.y() = temp.y();
			}
		}

	}
	return max;
};


void getPort(oaPoint Buoy, macro aMacro, oaPoint *Port){
    //input varibles
	oaCoord x_buoy=Buoy.x();
	oaCoord y_buoy=Buoy.y();
	oaCoord x_max=aMacro.r;
	oaCoord x_min=aMacro.l;
	oaCoord y_max=aMacro.t;
	oaCoord y_min=aMacro.b;

	  //output varibles
	oaCoord x_port, y_port;
	
	  //other varibles
	int x_o=(x_max-x_min)/2, y_o=(y_max-y_min)/2;
	float slope=(y_max-y_min)/(x_max-x_min);

    //Buoy is outside macro
	if (x_buoy>=x_max||x_buoy<=x_min||y_buoy>=y_max||y_buoy<=y_min) {
		if(x_buoy<x_min) x_port=x_min;
		if(x_buoy>=x_min&&x_buoy<=x_max) x_port=x_buoy;
		if(x_buoy>x_max) x_port=x_max;
		if(y_buoy<y_min) y_port=y_min;
		if(y_buoy>=y_min&&y_buoy<=y_max) y_port=y_buoy;
		if(y_buoy>y_max) y_port=y_max;
	}

    //Buoy is inside macro
	else {
		if(((y_buoy-y_o)>=slope*(x_buoy-x_o))&&((y_buoy-y_o)>=slope*(x_o-x_buoy))) {
			x_port=x_buoy;
			y_port=y_max;
		}
		if(((y_buoy-y_o)<slope*(x_buoy-x_o))&&((y_buoy-y_o)>=slope*(x_o-x_buoy))) {
			x_port=x_max;
			y_port=y_buoy;
		}
		if(((y_buoy-y_o)<slope*(x_buoy-x_o))&&((y_buoy-y_o)<slope*(x_o-x_buoy))) {
			x_port=x_buoy;
			y_port=y_min;
		}
		if(((y_buoy-y_o)>=slope*(x_buoy-x_o))&&((y_buoy-y_o)<slope*(x_o-x_buoy))) {
			x_port=x_min;
			y_port=y_buoy;
		}
	}

	Port->x() = x_port;
	Port->y() = y_port;
	
};


void getAim(oaInstTerm *instTermP , oaPoint *Port, macro *pMacroArray, oaPoint *Aim, oaPoint *LegalAim, bool *clkwise, int pertu){
    //input varibles
	macro  *theMacro = onWhichOne(pMacroArray, instTermP);
	oaPoint Initial = getIT_Pos(instTermP);
	oaCoord x_port=Port->x();
	oaCoord y_port=Port->y();
	oaCoord x_initial=Initial.x();
	oaCoord y_initial=Initial.y();
	oaCoord x_max=theMacro->r;
	oaCoord x_min=theMacro->l;
	oaCoord y_max=theMacro->t;
	oaCoord y_min=theMacro->b;
	oaCoord xStart=theMacro->xStart;
	oaCoord yStart=theMacro->yStart;
	

	//outut varibles
	oaCoord x_aim, y_aim, x_aim_legal, y_aim_legal;
	bool clkwise1=false, clkwise2=false;
	
	//find aim varibles
	int dist, dist_b; //move right up or left down; 
	oaCoord x_temp1, y_temp1, x_temp2, y_temp2;	
	int d_x=(x_initial>x_port)? (x_initial-x_port):(x_port-x_initial);
	int d_y=(y_initial>y_port)? (y_initial-y_port):(y_port-y_initial);	
	
	//legalization varibles
	//leagal coordinate closest to endpoints
	oaCoord x_min_legal=theMacro->l_legal;
	oaCoord x_max_legal=theMacro->r_legal;
	oaCoord y_min_legal=theMacro->b_legal;
	oaCoord y_max_legal=theMacro->t_legal;

	//condition that initial point must be on marco 
	bool condition_1=(x_initial==x_max||x_initial==x_min)&&(y_initial<=y_max)&&(y_initial>=y_min);
	bool condition_2=(y_initial==y_max||y_initial==y_min)&&(x_initial<=x_max)&&(x_initial>=x_min);  

	//check if the initial point is on macro
    if(condition_1||condition_2){

   	    //Case 1a. initial point and port point are on opposite sides(width)
    	if(d_x==(x_max-x_min)){
			
			//if can be moved to opposite side
			if(pertu>=d_x){
				*clkwise=(((y_initial>y_port)^(x_initial==x_max))?true:false);
				if(pertu<(d_x+d_y)){
					x_temp1=x_port;
					y_temp1=(y_initial>y_port)? (y_initial-pertu+d_x):(y_initial+pertu-d_x);
				}
				else{
					x_temp1=x_port;
					y_temp1=y_port;	
				}
				x_temp2=x_temp1;
				y_temp2=y_temp1;
			}
			
			else{	
				clkwise1=((x_initial==x_min)?true:false);
				clkwise2=((x_initial==x_max)?true:false);
				//start journey upward to port point and check pertubation at any moment
				if(pertu<(y_max-y_initial)){
					x_temp1=x_initial;
					y_temp1=y_initial+pertu;
				}
				else{
					x_temp1=x_initial+((x_initial==x_min)?(pertu-(y_max-y_initial)):((y_max-y_initial)-pertu));
					y_temp1=y_max;
				}
				
						
				//or start journey downward to port point and check pertubation at any moment
				if(pertu<(y_initial-y_min)){
					x_temp2=x_initial;
					y_temp2=y_initial-pertu;
				}
				else{
					x_temp2=x_initial+((x_initial==x_min)?(pertu-(y_initial-y_min)):((y_initial-y_min)-pertu));
					y_temp2=y_min;
				}
			}	
		}


		//Case 1b. initial point and port point are on opposite sides(length)
        else if(d_y==(y_max-y_min)){
			
			//if can be moved to opposite side
			if(pertu>=d_y){
				*clkwise=(((x_initial>x_port)^(y_initial==y_min))?true:false);
				if(pertu<(d_y+d_x)){
					y_temp1=y_port;
					x_temp1=(x_initial>x_port)? (x_initial-pertu+d_y):(x_initial+pertu-d_y);
				}
				else{
					y_temp1=y_port;
					x_temp1=x_port;	
				}
				y_temp2=y_temp1;
				x_temp2=x_temp1;
			}
			
			else{
				clkwise1=((y_initial==y_max)?true:false);
				clkwise2=((y_initial==y_min)?true:false);
				//start journey rightward to port point and check pertubation at any moment
				if(pertu<(x_max-x_initial)){
					y_temp1=y_initial;
					x_temp1=x_initial+pertu;
				}
				else{
					y_temp1=y_initial+((y_initial==y_min)?(pertu-(x_max-x_initial)):((x_max-x_initial)-pertu));
					x_temp1=x_max;
				}


				//or start journey leftward to port point and check pertubation at any moment
				if(pertu<(x_initial-x_min)){
					y_temp2=y_initial;
					x_temp2=x_initial-pertu;
				}
				else{
					y_temp2=y_initial+((y_initial==y_min)?(pertu-(x_initial-x_min)):((x_initial-x_min)-pertu));
					x_temp2=x_min;
				}
			}
        }


        //Case 2a. initial point and port point are on same side(width) (only one possible shortest way)
        else if(d_x==0){
		    if(y_port>y_initial){                      			
			    *clkwise=((x_initial==x_min)?true:false);
		    	if(pertu<(y_port-y_initial)){
		    		x_temp1=x_port;
		    		y_temp1=y_initial+pertu;
		    	}
		    	else{
				    x_temp1=x_port;
				    y_temp1=y_port; 
				}
			}   
			else{
				*clkwise=((x_initial==x_max)?true:false);
				if(pertu<(y_initial-y_port)){
					x_temp1=x_port;
					y_temp1=y_initial-pertu;
				}
				else{
				    x_temp1=x_port;
				    y_temp1=y_port;    
				}
			}
			x_temp2=x_temp1;
			y_temp2=y_temp1;
		}		


        //Case 2b. initial point and port point are on same side(length)  (only one possible shortest way)
		else if(d_y==0){
			if(x_port>x_initial){                                 
				*clkwise=((y_initial==y_max)?true:false);
				if(pertu<(x_port-x_initial)){
					y_temp1=y_port;
					x_temp1=x_initial+pertu;
				}
				else{
				    y_temp1=y_port;
				    x_temp1=x_port; 
				}
			}
			else{
				*clkwise=((y_initial==y_min)?true:false);
				if(pertu<(x_initial-x_port)){
					y_temp1=y_port;
					x_temp1=x_initial-pertu;
				}
				else{
				    y_temp1=y_port;
				    x_temp1=x_port;    
				} 
			}
			x_temp2=x_temp1;
			y_temp2=y_temp1;
		}	   



	    //Case 3. initial point and port point are on adjacent sides  (only one possible shortest way)
		else{

		    //for initial point on width
			if(x_initial==x_min||x_initial==x_max){
				if(y_port>y_initial){                                
				    *clkwise=((x_initial==x_min)?true:false);
					if(pertu<(y_max-y_initial)){
						x_temp1=x_initial;
						y_temp1=y_initial+pertu;
					}
					else if(pertu<(y_max-y_initial+d_x)){
						x_temp1=x_initial+((x_initial==x_min)?(pertu-(y_max-y_initial)):((y_max-y_initial)-pertu));
						y_temp1=y_max;
					}
					else{
						x_temp1=x_port;
						y_temp1=y_port;                                     
					}
				}
				else{
					*clkwise=((x_initial==x_max)?true:false);
				    if(pertu<(y_initial-y_min)){                     
				    	x_temp1=x_initial;
				    	y_temp1=y_initial-pertu;
				    }
				    else if(pertu<(y_initial-y_min+d_x)){
				    	x_temp1=x_initial+((x_initial==x_min)?(pertu-(y_initial-y_min)):((y_initial-y_min)-pertu));
				    	y_temp1=y_min;
				    } 
				    else{
					    x_temp1=x_port;
						y_temp1=y_port;
					}
				}
			}

		    //for initial point on length
			else if(y_initial==y_min||y_initial==y_max){
				if(x_port>x_initial){                             
					*clkwise=((y_initial==y_max)?true:false);
					if(pertu<(x_max-x_initial)){
						y_temp1=y_initial;
						x_temp1=x_initial+pertu;
					}
					else if(pertu<(x_max-x_initial+d_y)){
						y_temp1=y_initial+((y_initial==y_min)?(pertu-(x_max-x_initial)):((x_max-x_initial)-pertu));
						x_temp1=x_max;
					}
					else{
						y_temp1=y_port;
						x_temp1=x_port;
					}                 
				}
                else{                                             
                	*clkwise=((y_initial==y_min)?true:false);
					if(pertu<(x_initial-x_min)){
                		y_temp1=y_initial;
                		x_temp1=x_initial-pertu;
                	}
                	else if(pertu<(x_initial-x_min+d_y)){
                		y_temp1=y_initial+((y_initial==y_min)?(pertu-(x_initial-x_min)):((x_initial-x_min)-pertu));
                		x_temp1=x_min;
                	}
                	else{
						y_temp1=y_port;
						x_temp1=x_port;
					}
				}
			}
			x_temp2=x_temp1;
			y_temp2=y_temp1;			
		}
	}
	else{
		std::cerr<<"Initial points must be on the boundry of macro!"<<std::endl;
	}

	dist=((x_temp1>x_port)? (x_temp1-x_port):(x_port-x_temp1))+((y_temp1>y_port)? (y_temp1-y_port):(y_port-y_temp1));
	dist_b=((x_temp2>x_port)? (x_temp2-x_port):(x_port-x_temp2))+((y_temp2>y_port)? (y_temp2-y_port):(y_port-y_temp2));
	
	x_aim = ((dist<dist_b)?x_temp1:x_temp2);
	y_aim = ((dist<dist_b)?y_temp1:y_temp2);
	
	if(x_temp1!=x_temp2||y_temp1!=y_temp2) *clkwise=((x_aim==x_temp1)?clkwise1:clkwise2);

    Aim->x() = x_aim;
	Aim->y() = y_aim;
	
	//legalize Aim: divide macro into 8 parts
	oaCoord getDownLeftLegal(oaCoord illegal, oaCoord ref);
	oaCoord getUpRightLegal(oaCoord illegal, oaCoord ref);
	
    if(x_aim<x_min_legal&&y_aim>y_max_legal){
		x_aim_legal=(*clkwise==true)?x_min:x_min_legal;
		y_aim_legal=(*clkwise==true)?y_max_legal:y_max;
	}
	else if(x_aim>x_min_legal&&x_aim<x_max_legal&&y_aim==y_max){
		x_aim_legal=(*clkwise==true)?getDownLeftLegal(x_aim, xStart):getUpRightLegal(x_aim, xStart);
		y_aim_legal= y_max;
	}
	else if(x_aim>x_max_legal&&y_aim>y_max_legal){
		x_aim_legal=(*clkwise==true)?x_max_legal:x_max;
		y_aim_legal=(*clkwise==true)?y_max:y_max_legal;
	}
	else if(x_aim==x_max&&y_aim<y_max_legal&&y_aim>y_min_legal){
		x_aim_legal=x_max;
		y_aim_legal=(*clkwise==true)?getUpRightLegal(y_aim, yStart):getDownLeftLegal(y_aim, yStart);
	}
	else if(x_aim>x_max_legal&&y_aim<y_min_legal){
		x_aim_legal=(*clkwise==true)?x_max:x_max_legal;
		y_aim_legal=(*clkwise==true)?y_min_legal:y_min;
	}
    else if(x_aim>x_min_legal&&x_aim<x_max_legal&&y_aim==y_min){
		x_aim_legal=(*clkwise==true)?getUpRightLegal(x_aim, xStart):getDownLeftLegal(x_aim, xStart);
		y_aim_legal= y_min;
	}
    else if(x_aim<x_min_legal&&y_aim<y_min_legal){
        x_aim_legal=(*clkwise==true)?x_min_legal:x_min;
		y_aim_legal=(*clkwise==true)?y_min:y_min_legal;	
    }
    else if(x_aim==x_min&&y_aim<y_max_legal&&y_aim>y_min_legal){
        x_aim_legal=x_min;
		y_aim_legal=(*clkwise==true)?getDownLeftLegal(y_aim, yStart):getUpRightLegal(y_aim, yStart);
	}
	else{
		x_aim_legal=x_aim;
		y_aim_legal=y_aim;
	}
	
	LegalAim->x() = x_aim_legal;
	LegalAim->y() = y_aim_legal;
	
};


oaCoord getDownLeftLegal(oaCoord illegal, oaCoord ref){
	oaCoord legal;
	if(((illegal>ref)?(illegal-ref):(ref-illegal))%pinMoveStep==0) legal=illegal;
	else {
		legal=illegal - ((illegal>ref)?((illegal-ref)%pinMoveStep):(pinMoveStep-((ref-illegal)%pinMoveStep)));
	}	
    return legal;
};


oaCoord getUpRightLegal(oaCoord illegal, oaCoord ref){
	oaCoord legal;
	if(((illegal>ref)?(illegal-ref):(ref-illegal))%pinMoveStep==0) legal=illegal;
	else {
		legal=illegal + ((illegal>ref)?(pinMoveStep-((illegal-ref)%pinMoveStep)):((ref-illegal)%pinMoveStep));
	}	
    return legal;
};


oaCoord calManhattan(oaPoint point_1, oaPoint point_2){	
	oaCoord x_1=point_1.x();
	oaCoord y_1=point_1.y();
	oaCoord x_2=point_2.x();
	oaCoord y_2=point_2.y();
	oaCoord Manha=0;
	Manha=((x_1>x_2)? (x_1-x_2):(x_2-x_1)) + ((y_1>y_2)? (y_1-y_2):(y_2-y_1));
	return Manha;	
};


int isOccupied(macro * tempMacro, oaPoint* Aim, bool isOverPert){
	bool checkNeighbour(oaPoint point, side Side,side Side2, int side, macro tempMacro);
	std::set<oaCoord>::iterator it;
	it = tempMacro->lside.list.find(Aim->y());
	if (Aim->x() == tempMacro->l && it == tempMacro->lside.list.end() && Aim->y()!= tempMacro->t && Aim->y()!= tempMacro->b)
	{	if (checkNeighbour(*Aim, tempMacro->lside,tempMacro->lside, 1 ,* tempMacro) == 1 && (isOverPert == 0))
		{
		tempMacro->lside.list.insert(Aim->y());
		return 0;
		}
		else{
		return 1;
		}

	}else if (Aim->x() == tempMacro->l && it != tempMacro->lside.list.end()){
		return 1;
	}
	it = tempMacro->rside.list.find(Aim->y());
	if (Aim->x() == tempMacro->r && it == tempMacro->rside.list.end() && Aim->y()!= tempMacro->t && Aim->y()!= tempMacro->b)
	{
		if (checkNeighbour(*Aim, tempMacro->rside, tempMacro->rside, 2 , * tempMacro) == 1 && (isOverPert == 0))
		{
		tempMacro->rside.list.insert(Aim->y());
		return 0;
		}
		else{
		return 2;}
	}else if (Aim->x() == tempMacro->r && it != tempMacro->rside.list.end()){
		return 2;
	}
	it = tempMacro->tside.list.find(Aim->x());
	if (Aim->y() == tempMacro->t && it == tempMacro->tside.list.end()) 
	{
		if (checkNeighbour(*Aim, tempMacro->tside,tempMacro->tside, 3 ,* tempMacro) == 1 && (isOverPert == 0))
		{
		tempMacro->tside.list.insert(Aim->x());
		return 0;
		}
		else{
		return 3;
		}
	}else if (Aim->y() == tempMacro->t && it != tempMacro->tside.list.end()){
		return 3;
	}
	it = tempMacro->bside.list.find(Aim->x());
	if (Aim->y() == tempMacro->b && it == tempMacro->bside.list.end())
	{
		if (checkNeighbour(*Aim, tempMacro->bside,tempMacro->bside, 4 ,* tempMacro) == 1 && (isOverPert == 0))
		{
		tempMacro->bside.list.insert(Aim->x());
		return 0;
		}
		else{
		return 4;
		}
	}else if (Aim->y() == tempMacro->b && it != tempMacro->bside.list.end()){
		return 4;
	}
};


int isNearTheCorner(oaPoint* pPoint, macro * pM,side * pSide,int side, bool clkwise){
	if (clkwise == 1)
	{
		if ((pPoint->y() > pM->t_legal) && (side == 1))
		{
			pPoint->y() = pM->t;
			pPoint->x() = pM->l_legal;
			return 3;
		}
		if ((pPoint->y() < pM->b_legal)  && (side == 2))
		{
			pPoint->y() = pM->b;
			pPoint->x() = pM->r_legal;
			return 4;
		}
		if ((pPoint->x() > pM->r_legal) && (side == 3))
		{
			pPoint->x() = pM->r;
			pPoint->y() = pM->t_legal;
			return 2;
		}
		if ((pPoint->x() < pM->l_legal) && (side == 4))
		{
			pPoint->x() = pM->l;
			pPoint->y() = pM->b_legal;
			return 1;
		}  
	}	
	else if (clkwise == 0)
	{
		if ((pPoint->y()< pM->b_legal) && (side == 1))
		{
			pPoint->y() = pM->b;
			pPoint->x() = pM->l_legal;
			return 4;
		}
		if ((pPoint->y()  > pM->t_legal) && (side == 2))
		{
			pPoint->y() = pM->t;
			pPoint->x() = pM->r_legal;
			return  3;
		}
		if ((pPoint->x() > pM->r_legal) && (side == 4))
		{
			pPoint->x() = pM->r;
			pPoint->y() = pM->b_legal;
			return 2;
		}
		if ((pPoint->x() < pM->l_legal) && (side == 3))
		{
			pPoint->x() = pM->l;
			pPoint->y() = pM->t_legal;
			return  1;
		}
	}
		return 0;
};


void moveAlongSide(oaPoint * LegalAim, int side ,bool clkwise){
	if (clkwise == 1 )
	{
		switch (side){
			case 1 : LegalAim->y() = LegalAim->y() + pinMoveStep;break;
			case 2 : LegalAim->y() = LegalAim->y() - pinMoveStep;break;
			case 3 : LegalAim->x() = LegalAim->x() + pinMoveStep;break;
			case 4 : LegalAim->x() = LegalAim->x() - pinMoveStep;break;
		}
	}else{
		switch (side){
			case 1 : LegalAim->y() = LegalAim->y() - pinMoveStep;break;
			case 2 : LegalAim->y() = LegalAim->y() + pinMoveStep;break;
			case 3 : LegalAim->x() = LegalAim->x() - pinMoveStep;break;
			case 4 : LegalAim->x() = LegalAim->x() + pinMoveStep;break;
		}
	}
};


bool checkNeighbour(oaPoint point, side Side, side Side2, int side, macro tempMacro){
	oaPoint plus = point;
	oaPoint minus = point;
	int currentSide;
	int side2=side;
	for (int i = 0; i < range-1; ++i)
	{
			//clkwise 
			std::set<oaCoord>::iterator it;
			currentSide = isNearTheCorner(&plus, &tempMacro, &Side, side, 1);
			if (currentSide == 0 )
			{
				//same side
				currentSide = side;
			}else if (currentSide != side && currentSide != 0){

				//change side
				side = currentSide;
			}
			if (currentSide == 1 )
			{  			
				plus.y() = plus.y() + pinMoveStep;
				it = tempMacro.lside.list.find(plus.y());
				Side = tempMacro.lside;				
			}
			if (currentSide == 2 )
			{	 
				plus.y() = plus.y() - pinMoveStep;
				it =  tempMacro.rside. list.find(plus.y());
				Side = tempMacro.rside;			
			}
			if (currentSide == 3 )
			{	
				plus.x() = plus.x() + pinMoveStep;
				it =  tempMacro.tside.list.find(plus.x());			
				Side = tempMacro.tside;	
			}
			if (currentSide == 4 )
			{ 
				plus.x() = plus.x() - pinMoveStep;	
				it =  tempMacro.bside.list.find(plus.x());
				Side = tempMacro.bside;		
			}
			if (*it != Side.list.size() && Side.list.empty()!= 1 )
			{
				//find!!!
				//violation
				return 0;
			}
	}
	for (int i = 0; i < range-1; ++i)
	{	 
			//c-clkwise 
			std::set<oa::oaCoord>::iterator it;
			currentSide = isNearTheCorner(&minus, &tempMacro, &Side2, side2, 0);
			if (currentSide == 0)
			{
				//same side
				currentSide = side2;
			}else if (currentSide != side&& currentSide != 0){
				//change side
				side2 = currentSide;
			}
			if (currentSide == 1 )
			{
				minus.y() = minus.y() - pinMoveStep;
				it =  tempMacro.lside . list.find(minus.y());
				Side2 = tempMacro.lside;
				
			}
			else if (currentSide == 2 )
			{
				minus.y() = minus.y() + pinMoveStep;
				it =  tempMacro.rside .list.find(minus.y());
				Side2 = tempMacro.rside;
				
			}
			else if (currentSide == 3 )
			{
				minus.x() = minus.x() - pinMoveStep;
				it =  tempMacro.tside .list.find(minus.x());
				Side2 = tempMacro.tside;
				
			}
			else if (currentSide == 4 )
			{
				minus.x() = minus.x() + pinMoveStep;
				it =  tempMacro.bside . list.find(minus.x());
				
				Side2 = tempMacro.bside;
			}
			if (*it != Side2.list.size() && Side2.list.empty() != 1)
			{
				//find!!!
				//violation
				return 0;
			}
	}
	return 1;
};


void eraseAim(macro *tempMacro, oaPoint* Aim){
	if(Aim->x()==tempMacro->l)
		tempMacro->lside.list.erase(Aim->y());
	else if(Aim->x()==tempMacro->r)
		tempMacro->rside.list.erase(Aim->y());
	else if(Aim->y()==tempMacro->t)
		tempMacro->tside.list.erase(Aim->x());
	else if(Aim->y()==tempMacro->b)
		tempMacro->bside.list.erase(Aim->x());
};


int MoveLegalAim(oaPoint *LegalAim, macro *tempMacro, int currentSide, bool isOverPert, bool clkwise){
	int isNearTheCorner(oaPoint* pPoint, macro * pM,side * pSide,int side, bool clkwise);
	void moveAlongSide(oaPoint * LegalAim, int side ,bool clkwise);
	int isOccupied(macro * tempMacro, oaPoint* Aim, bool isOverPert);
	side * pcurrentSide;
	switch (currentSide){
		case 1: pcurrentSide = & tempMacro->lside;break;
		case 2: pcurrentSide = & tempMacro->rside;break;
		case 3: pcurrentSide = & tempMacro->tside;break;
		case 4: pcurrentSide = & tempMacro->bside;break;
	}
	int newSide = isNearTheCorner(LegalAim, tempMacro, pcurrentSide, currentSide, clkwise);
	if ((newSide != currentSide) && (newSide != 0))
	{	
		//move to next side
		currentSide = isOccupied(tempMacro, LegalAim, isOverPert);
	}
	else if (newSide ==0)
	{
		//start at the same side
		//move along the side by pinMoveStep
		moveAlongSide(LegalAim, currentSide ,clkwise);
		newSide =  isNearTheCorner(LegalAim, tempMacro, pcurrentSide, currentSide, clkwise);
		if (newSide == 0)
		{
			currentSide = isOccupied(tempMacro, LegalAim, isOverPert);
		}
		else{
			currentSide = newSide;
		}
	}
	return currentSide;
};