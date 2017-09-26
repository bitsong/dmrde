#ifndef _BIT_REGION
#define _BIT_REGION


#define __region_caculate(num,op,x,mask) 	do{(num) = (((num) op (x)) & (mask)) | ((num) & (~(mask)));}while(0)

#define _region_set(num,x,mask)				do{(num) = ((num) & (~(mask))) | ((x) & (mask));}while(0)	

#define _region_add(num,x,mask)				__region_caculate(num,+,x,mask)

#define _region_sub(num,x,mask)				__region_caculate(num,-,x,mask)

	
#define bits_set(num,bits,mask)				do{(num) |= ((mask) & (bits));}while(0)

#define bits_clear(num,bits,mask)			do{(num) &= ~((mask) & (bits));}while(0)

#define bits_test(num,bits,mask)			((num) & ((bits) & (mask)))


#define region_set(num,x,xmask,xshift)		_region_set(num,(x) << (xshift),xmask)

#define region_add(num,x,xmask,xshift)		_region_add(num,(x) << (xshift),xmask)

#define region_sub(num,x,xmask,xshift)		_region_sub(num,(x) << (xshift),xmask)

#define region_get(num,mask,shift)			(((num) & (mask)) >> (shift))

#define region_clear(num,mask)				do{(num) &= ~(mask);}while(0)

#define region_compare(num,x,mask,shift)	(region_get(num,mask,shift) > (x))// 1:num 中的值 > x , 0:num中的值 < x

//#define region_compare(num,mask,shift,x)	(region_get(num,mask,shift) > (x))

#define REGION(start,bits)					(((1 << ((start) + (bits))) - 1) & (~((1 << (start) ) - 1)))

#define REGION_FLAG_BIT(region,start,pos)	((1 << ((start) + (pos))) & (region))

/*

#define REGION(start,bits)					(start),(bits)
#define REGION_MASK(start,bits)				(((1 << ((start) + (bits))) - 1) & (~((1 << (start) ) - 1)))

#define region_set(num,region,val)			region_set(num,val,region)
#define region_get(num,region)				region_get(num,region)
#define region_add(num,region,val)			region_add(num,val,region)
#define region_sub(num,region,val)			region_sub(num,val,region)
#define region_clear(num,region)			region_clear(num,region)
#define region_compare(num,region,val)		region_compare(num,val,region)

#define bits_set()
#define bits_clear()
#define bits_test()

 */

#endif

