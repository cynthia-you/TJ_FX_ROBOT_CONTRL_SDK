#include "ShMem.h"


void SHMOnUnMap(void* object_ptr)
{
	ShMem* pcast = (ShMem*)object_ptr;
#ifdef CMPL_WIN
	if(pcast->m_shmem != NULL)
	{
		pcast->m_shmem -= 8;
		UnmapViewOfFile(pcast->m_shmem);
		pcast->m_shmem = NULL;
		pcast->m_shm_size = 0;
	}
    if(pcast->is_master)
	{
    	if(pcast->m_hMapFile != NULL)
    	{
    		CloseHandle(pcast->m_hMapFile);
    		pcast->m_hMapFile = NULL;
    	}
    }
#endif

#ifdef CMPL_LIN
	if(pcast->m_shmem != NULL)
	{
		pcast->m_shmem -= 8;
		shmdt(pcast->m_shmem);
		pcast->m_shmem = NULL;
	}
    if(pcast->is_master)
	{
    	if(pcast->shmid != -1)
    	{
    		shmctl(pcast->shmid, IPC_RMID, 0);
    		pcast->shmid = -1;
    	}
    }
#endif
}

FX_BOOL SHMOnMapMster(void* object_ptr, FX_CHAR* shm_path_ptr, FX_UINT32 size)
{
	ShMem* pcast = (ShMem*)object_ptr;
	SHMOnUnMap(object_ptr);
	if(size < 1)
	{
		return FX_FALSE;
	}
	
#ifdef CMPL_WIN
	pcast->m_hMapFile = OpenFileMapping(FILE_MAP_ALL_ACCESS, 0, shm_path_ptr);
	if(pcast->m_hMapFile != NULL)
	{
		CloseHandle(pcast->m_hMapFile);
		return FX_FALSE;
	}
	pcast->m_hMapFile = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, size+8, shm_path_ptr); 
	if(pcast->m_hMapFile == NULL)
	{
		DWORD ev = GetLastError();
		//printf("eno: %d\n",ev);//ERROR_INVALID_HANDLE
		return FX_FALSE;
	}
	
	pcast->m_shmem = (FX_UCHAR*)MapViewOfFile(pcast->m_hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, 0);
	FX_INT32* psize = (FX_INT32*)pcast->m_shmem;
	*psize = size;
	pcast->m_shm_size = size;      
	pcast->m_shmem = &pcast->m_shmem[8];
#endif
#ifdef CMPL_LIN 
	key_t kt = ftok(shm_path_ptr, 7);
	if(kt == -1)
	{
		return FX_FALSE;
	}
	pcast->shmid = shmget(kt, size + 8 , 0666 | IPC_CREAT);  
	if(pcast->shmid == -1)
	{
		return FX_FALSE;
	}
	
	pcast->m_shmem = (FX_UCHAR*)shmat(pcast->shmid, (FX_VOID*)0, 0);
	if(pcast->m_shmem == (void*)-1)
	{
		return FX_FALSE;
	}
	
	FX_INT32* psize = (FX_INT32*)pcast->m_shmem;
	*psize = size;
	pcast->m_shm_size = size;
	pcast->m_shmem = &pcast->m_shmem[8];
#endif
    pcast->is_master = FX_TRUE;
	return FX_TRUE;
}

FX_BOOL SHMOnMapSlave(FX_VOID* object_ptr, FX_CHAR* shm_path_ptr)
{
	ShMem* pcast = (ShMem*)object_ptr;
	SHMOnUnMap(object_ptr);
	
#ifdef CMPL_WIN
	pcast->m_hMapFile = OpenFileMapping(FILE_MAP_ALL_ACCESS, 0, shm_path_ptr);
	if(pcast->m_hMapFile == NULL)
	{
		return FX_FALSE;
	}
	pcast->m_shmem = (FX_UCHAR*)MapViewOfFile(pcast->m_hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, 0);
	FX_INT32* psize = (FX_INT32*)pcast->m_shmem;
	pcast->m_shm_size = *psize; 
	pcast->m_shmem = &pcast->m_shmem[8];
#endif
	
#ifdef CMPL_LIN 	
	key_t kt = ftok(shm_path_ptr, 7);
	if(kt == -1)
	{
		return FX_FALSE;
	}
	pcast->shmid = shmget(kt, 0, 0);  
	if(pcast->shmid == -1)
	{
		return FX_FALSE;
	}
	
	pcast->m_shmem = (FX_UCHAR*)shmat(pcast->shmid, (FX_VOID*)0, 0);
	if(pcast->m_shmem == (FX_VOID*)-1)
	{
		return FX_FALSE;
	}
	FX_INT32* psize = (FX_INT32*)pcast->m_shmem;
	pcast->m_shm_size = *psize;       
	pcast->m_shmem = &pcast->m_shmem[8];	
#endif
    pcast->is_master = FX_FALSE;
	return FX_TRUE;
}

FX_UCHAR* SHMOnGetMem(FX_VOID* object_ptr)
{
	ShMem* pcast = (ShMem*)object_ptr;
	return pcast->m_shmem;
}

FX_INT32 SHMOnGetMapSize(FX_VOID* object_ptr)
{	
	ShMem* pcast = (ShMem*)object_ptr;
	return pcast->m_shm_size;
}

FX_VOID SHMOnDest(FX_VOID* object_ptr)
{
	SHMOnUnMap(object_ptr);
	ShmOnInit(object_ptr);
}

FX_VOID ShmOnInit(FX_VOID* object_ptr)
{
	
	ShMem* pcast = (ShMem*)object_ptr;
	pcast->m_shmem = NULL;
	pcast->m_shm_size = 0;
#ifdef CMPL_WIN
	pcast->m_hMapFile = NULL;
#endif
#ifdef CMPL_LIN
	pcast->shmid = -1;
#endif
    pcast->is_master = FX_FALSE;
	pcast->OnDest = SHMOnDest;
	pcast->OnMapMster = SHMOnMapMster;
	pcast->OnMapSlave = SHMOnMapSlave;
	pcast->OnGetMem = SHMOnGetMem;
	pcast->OnGetMapSize = SHMOnGetMapSize;
}

