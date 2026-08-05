#include "cache.h"

// initialize replacement state
void CACHE::llc_initialize_replacement()
{

}

// find replacement victim
uint32_t CACHE::llc_find_victim(uint32_t cpu, uint64_t instr_id, uint32_t set, const BLOCK *current_set, uint64_t ip, uint64_t full_addr, uint32_t type)
{
    // baseline LRU
    return lru_victim(cpu, instr_id, set, current_set, ip, full_addr, type); 
}

// called on every cache hit and cache fill
void CACHE::llc_update_replacement_state(uint32_t cpu, uint32_t set, uint32_t way, uint64_t full_addr, uint64_t ip, uint64_t victim_addr, uint32_t type, uint8_t hit)
{
    string TYPE_NAME;
    if (type == LOAD)
        TYPE_NAME = "LOAD";
    else if (type == RFO)
        TYPE_NAME = "RFO";
    else if (type == PREFETCH)
        TYPE_NAME = "PF";
    else if (type == WRITEBACK)
        TYPE_NAME = "WB";
    else
        assert(0);

    if (hit)
        TYPE_NAME += "_HIT";
    else
        TYPE_NAME += "_MISS";

    if ((type == WRITEBACK) && ip)
        assert(0);

    // uncomment this line to see the LLC accesses
    // cout << "CPU: " << cpu << "  LLC " << setw(9) << TYPE_NAME << " set: " << setw(5) << set << " way: " << setw(2) << way;
    // cout << hex << " paddr: " << setw(12) << paddr << " ip: " << setw(8) << ip << " victim_addr: " << victim_addr << dec << endl;

    // baseline LRU
    if (hit && (type == WRITEBACK)) // writeback hit does not update LRU state
        return;

    return lru_update(set, way);
}

void CACHE::llc_replacement_final_stats()  //guru
{
        cout<<"The no. of writes in each cache line are: "<<endl;
    int wc=0;
    for(int i=0;i<LLC_WAY;i++)
     cout<<"\t"<<i;
     cout<<"\tSum";
     cout<<endl;
     for(int i=0;i<LLC_SET;i++)
     {
        cout<<i<<" ";
        for(int j=0;j<LLC_WAY;j++)
        {
            cout<<"\t"<<writes_set[i][j];
            wc+=writes_set[i][j];
        }
        cout<<"\t"<<wc;
        wc=0;
        cout<<endl;
     }

    double averageWritesPerSet[LLC_SET];
    	for(int i=0;i<LLC_SET;i++){
    		double average = 0.0;
    		for(int j=0;j<LLC_WAY;j++){
    			average = average + ((double)writes_set[i][j]);
    		}
    		average = average / LLC_WAY;
    		averageWritesPerSet[i] = average;
    	}

    	double finalAns = 0.0;

    	for(int i=0;i<LLC_SET;i++){
    		double var =0.0;
    		for(int j=0;j<LLC_WAY;j++){
    			var = var + (averageWritesPerSet[i]-writes_set[i][j]) * (averageWritesPerSet[i]-writes_set[i][j]);
    		}
    		var = var/(LLC_WAY-1);   //guru
    		var = sqrt(var);
    		finalAns =finalAns+var;
    	}

    	finalAns = finalAns / (LLC_SET);

    	//calculate average number of writes
    	double avgWrites = 0.0;
    	for(int i=0;i<LLC_SET;i++){
    		for(int j=0;j<LLC_WAY;j++){
    			avgWrites = avgWrites + writes_set[i][j];
    		}
    	}
    	avgWrites = avgWrites / LLC_WAY;
    	avgWrites = avgWrites / (LLC_SET);

    	finalAns = finalAns / avgWrites;

    	cout<<endl<<"-------------------------------------------------------------"<<endl;
    	cout<<endl<<"-------------------------------------------------------------"<<endl;
    	cout<<endl<<"-------------------------------------------------------------"<<endl;
    	cout<<endl<<"-------------------------------------------------------------"<<endl;
    	cout<<"Coefficient of intra set write Variation is -->  "<<finalAns;


}
