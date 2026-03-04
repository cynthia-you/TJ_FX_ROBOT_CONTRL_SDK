#ifndef FX_SHMEM_H_ 
#define FX_SHMEM_H_

/**
 * @file ShMem.h
 * @brief The file defines a share memory object.
 * @author an.jie
 */

#include "CmplOpt.h"
#include "FxType.h"


#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief Initialize an ShMem object.
 * @param [in] object_ptr Pointer of ShMem.
 */
FX_VOID ShmOnInit(FX_VOID* object_ptr);
/**
 * @brief Defines the class of general share memory.
 */
typedef struct
{
    /**
     * @brief Clean all internal data and release share memory.
     * @param [in] object_ptr Pointer of the object.
     */
	typedef FX_VOID (*SHMOnDestPtr)(FX_VOID* object_ptr);
    /**
     * @brief Map the object with a share memory on master side.
     * @param [in] object_ptr Pointer of the object.
     * @param [in] shm_path_ptr Absolute path of share memory.
     * @param [in] size Data section byte size of share memory.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */     
	typedef FX_BOOL (*SHMOnMapMsterPtr)(FX_VOID* object_ptr, FX_CHAR* shm_path_ptr, FX_UINT32 size);
    /**
     * @brief Map the object with a share memory on slave side.
     * @param [in] object_ptr Pointer of the object.
     * @param [in] shm_path_ptr Absolute path of share memory.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */ 
	typedef FX_BOOL (*SHMOnMapSlavePtr)(FX_VOID* object_ptr, FX_CHAR* shm_path_ptr);
    /**
     * @brief Get the base address of data section of the share memory.
     * @param [in] object_ptr Pointer of the object.
     * @return Pointer of the base address of data section.
     */    
	typedef	FX_UCHAR* (*SHMGetMemPtr)(FX_VOID* object_ptr);
    /**
     * @brief Get the byte size of the share memory.
     * @param [in] object_ptr Pointer of the object.
     * @return Byte size of the share memory.
     */     
	typedef	FX_INT32 (*SHMOnGetMapSizePtr)(FX_VOID* object_ptr);

	FX_UCHAR* m_shmem;     /**< Pointer of the base address of data section of the share memory.*/     
	FX_UINT32 m_shm_size;   /**< Byte size of data section.*/
	#ifdef CMPL_WIN
	HANDLE m_hMapFile;          /**< Share memory handler.*/
	#endif
	#ifdef CMPL_LIN
	FX_INT32 shmid;                  /**< Share memory id.*/
	#endif
    FX_BOOL is_master;

	SHMOnDestPtr OnDest;                /**< Function pointer of SHMOnDestPtr.*/
	SHMOnMapMsterPtr OnMapMster;        /**< Function pointer of SHMOnMapMsterPtr.*/
	SHMOnMapSlavePtr OnMapSlave;        /**< Function pointer of SHMOnMapSlavePtr.*/
	SHMGetMemPtr OnGetMem;              /**< Function pointer of SHMGetMemPtr.*/
	SHMOnGetMapSizePtr OnGetMapSize;    /**< Function pointer of SHMOnGetMapSizePtr.*/
}ShMem;

#ifdef __cplusplus
}
#endif

#endif 

