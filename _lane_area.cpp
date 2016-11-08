/*
 * lane_area.cpp
 *
 *  Created on: Mar 29, 2015
 *      Author: chengxian
 */

#include "_lane_area.h"


void LaneArea::detect_lane_marker_points_one_line(CamParam *pCamParam, int iIsrc, int iProcLineIndex, unsigned char *pImage, LaneMarkerPoints *pUpEdgePoints, LaneMarkerPoints *pDownEdgePoints)
{

	//这个不是很懂

	int	iPositiveThresholdLeft = this->get_region(CHX_NF_NEAR)->get_side(CHX_LR_LEFT)->get_threshold (CHX_UD_UP);
	int	iNegativeThresholdLeft = this->get_region(CHX_NF_NEAR)->get_side(CHX_LR_LEFT)->get_threshold(CHX_UD_DOWN);
	int	iPositiveThresholdRight =this->get_region(CHX_NF_NEAR)->get_side(CHX_LR_RIGHT)->get_threshold(CHX_UD_UP);
	int	iNegativeThresholdRight =this->get_region(CHX_NF_NEAR)->get_side(CHX_LR_RIGHT)->get_threshold(CHX_UD_DOWN);

	int iJsrcMin=this->get_Jsrc_left_at_Isrc_min();
	int iJsrcMax=this->get_Jsrc_left_at_Isrc_max();
	////////
	this->detectLaneMarkerPointsOneLine(pCamParam, iIsrc,iJsrcMin,iJsrcMax, iProcLineIndex, pImage, pUpEdgePoints, pDownEdgePoints,
			iPositiveThresholdLeft, iNegativeThresholdLeft, iPositiveThresholdRight, iNegativeThresholdRight);

}
bool LaneArea::detect_lane_marker_points(CamParam *pCamParam, unsigned char *pImage, LaneMarkerPoints *pUpEdgePoints, LaneMarkerPoints *pDownEdgePoints){

	if (pCamParam == NULL) {
		return false;
	}
	if (pImage == NULL) {
		return false;
	}
	if (pUpEdgePoints == NULL) {
		return false;
	}
	if (pDownEdgePoints == NULL) {
		return false;
	}

	int iProcLineNumber = this->get_process_line_number();
	for (int iIdx = iProcLineNumber - 1; iIdx >= 0; iIdx--) {
		int iIsrc = (this->get_process_line())[iIdx];
		//LARGE_INTEGER t1,t2,tc;
		//QueryPerformanceFrequency(&tc);
		//QueryPerformanceCounter(&t1);
		this->detect_lane_marker_points_one_line(pCamParam, iIsrc, iIdx, pImage,
			pUpEdgePoints, pDownEdgePoints);
		//Sleep(1000);
		//QueryPerformanceCounter(&t2);

		//cout<<"function [detect_lane_marker_points_one_line]: elapse_time="<<(t2.QuadPart - t1.QuadPart)*1.0/(tc.QuadPart)*1000<<" ms "<<endl;
	}

	return true;
}
bool LaneArea::detect_lane_markers(CamParam *pCamParam, LaneMarkerPoints *pUpEdgePoints, LaneMarkerPoints *pDownEdgePoints, unsigned char *pInputImage){

	if (pCamParam == NULL) {
		return false;
	}
	if (pUpEdgePoints == NULL) {
		return false;
	}
	if (pDownEdgePoints == NULL) {
		return false;
	}

	for (int iNF = 0; iNF < CHX_NF_NUM; iNF++) {
		LaneRegion *pRegion = this->get_region(iNF);
		if (pRegion == NULL)
			continue;
		//////////////
		pRegion->vote(pCamParam, pUpEdgePoints, pDownEdgePoints);/////////////////////////////////

		////////////////
		pRegion->search_lane_marker_lines();	////////

		/////////////////////
		int iProcLineIndexMin =
				(iNF == CHX_NF_NEAR) ?
						(this->get_line_number_far_area() + this->get_region(CHX_NF_FAR)-> get_line_number()) :
						this->get_line_number_far_area();
		int iProcLineIndexMax = iProcLineIndexMin + this->get_region(iNF)->get_line_number()
				- 1;
		//////////
		int iIsrcMin = (this->get_process_line())[iProcLineIndexMin];
		int iIsrcMax = (this->get_process_line())[iProcLineIndexMax];

		pRegion-> pickup_lane_marker_points_of_lane_marker_lines(pCamParam,
				pUpEdgePoints, pDownEdgePoints, iIsrcMin, iIsrcMax);
		//////////
		pRegion->calculate_average_edge_strength_of_lane_marker_lines();

		//////////////
		int aiTopIntensity[CHX_LR_NUM] = { this->get_top_intensity(CHX_LR_LEFT),
				this->get_top_intensity(CHX_LR_RIGHT) };
		pRegion->pair_lane_marker_lines(pCamParam, pInputImage, aiTopIntensity);
		//////////////
		pRegion->pair_lane_markers();
	}

	return true;
}

bool sortFlexArrayDouble(FlexArray<double> *pfa)
{
	if(pfa == NULL)	return false;
	for(int iIdx0 = 0; iIdx0 < pfa->get_number(); iIdx0++) {
		for(int iIdx1 = iIdx0 + 1; iIdx1 < pfa->get_number(); iIdx1++) {
			if(pfa->get(iIdx0) > pfa->get(iIdx1)) {
				double dTmp = pfa->get(iIdx0);
				pfa->set(iIdx0, pfa->get(iIdx1));
				pfa->set(iIdx1, dTmp);
			}
		}
	}
	return true;
}
LaneMarker *searchLaneMarkerForLaneBoundary(LaneMarkers *pLMs, int iUD, int iOffsetCenter, int iOffsetMin, int iOffsetMax, int iYawCenter, int iYawMin, int iYawMax)
{
	if(pLMs == NULL)	return NULL;

	LaneMarker *pLMSel = NULL;
	int iOffsetSel = -1;
	int iYawSel = 0;
	for(int iIdx = 0; iIdx < pLMs->get_lane_marker_number(); iIdx++) {
		LaneMarker *pLM = pLMs->get_lane_marker(iIdx);
		if(pLM == NULL)	continue;
		LaneMarkerLine *pLML = pLM->get_lane_marker_line(iUD);
		if(pLML == NULL)	continue;
		int iOffset = pLML->get_offset();
		int iYaw = pLML->get_yaw();
		if(iOffset < iOffsetMin)	continue;
		if(iOffset > iOffsetMax)	continue;
		if(iYaw < iYawMin)		continue;
		if(iYaw > iYawMax)		continue;
		if(pLMSel == NULL) {
			pLMSel = pLM;
			iOffsetSel = iOffset;
			iYawSel = iYaw;
		} else {
			if(abs(iOffsetSel - iOffsetCenter) < abs(iOffset - iOffsetCenter)) {
			} else if(abs(iOffsetSel - iOffsetCenter) > abs(iOffset - iOffsetCenter)) {
				pLMSel = pLM;
				iOffsetSel = iOffset;
				iYawSel = iYaw;
			} else {	// get_offset is equal
				if(abs(iYawSel - iYawCenter) > abs(iYaw - iYawCenter)) {
					pLMSel = pLM;
					iOffsetSel = iOffset;
					iYawSel = iYaw;
				} 
			}
		}
	}

	return pLMSel;
}
LaneMarker *searchLaneMarkerForLaneBoundaryByVotes(LaneMarkers *pLMs, int iUD, int iOffsetCenter, int iOffsetMin, int iOffsetMax, int iYawCenter, int iYawMin, int iYawMax)
{
	if(pLMs == NULL)	return NULL;

	LaneMarker *pLMSel = NULL;
	int iOffsetSel = -1;
	int iYawSel = 0;
	int iVotesSel = 0;
	for(int iIdx = 0; iIdx < pLMs->get_lane_marker_number(); iIdx++) {
		LaneMarker *pLM = pLMs->get_lane_marker(iIdx);
		if(pLM == NULL)	continue;
		LaneMarkerLine *pLML = pLM->get_lane_marker_line(iUD);
		if(pLML == NULL)	continue;
		int iOffset = pLML->get_offset();
		int iYaw = pLML->get_yaw();
		if(iOffset < iOffsetMin)	continue;
		if(iOffset > iOffsetMax)	continue;
		if(iYaw < iYawMin)		continue;
		if(iYaw > iYawMax)		continue;
		int iVotes = pLM->get_up()->get_votes() + pLM->get_down()->get_votes();
		if(pLMSel == NULL) {
			pLMSel = pLM;
			iOffsetSel = iOffset;
			iYawSel = iYaw;
			iVotesSel = iVotes;
		} else {
			if(iVotesSel < iVotes) {
				pLMSel = pLM;
				iOffsetSel = iOffset;
				iYawSel = iYaw;
				iVotesSel = iVotes;
			} else if(iVotesSel > iVotes) {
			} else {
				if(abs(iOffsetSel - iOffsetCenter) < abs(iOffset - iOffsetCenter)) {
				} else if(abs(iOffsetSel - iOffsetCenter) > abs(iOffset - iOffsetCenter)) {
					pLMSel = pLM;
					iOffsetSel = iOffset;
					iYawSel = iYaw;
					iVotesSel = iVotes;
				} else {	// get_offset is equal
					if(abs(iYawSel - iYawCenter) > abs(iYaw - iYawCenter)) {
						pLMSel = pLM;
						iOffsetSel = iOffset;
						iYawSel = iYaw;
						iVotesSel = iVotes;
					}
				} 
			}
		}
	}

	return pLMSel;
}
LaneMarkerLine *searchLaneMarkerLineForLaneBoundary(LaneMarkerLines *pLMLs, int iOffsetCenter, int iOffsetMin, int iOffsetMax, int iYawCenter, int iYawMin, int iYawMax)
{
	if(pLMLs == NULL)	return NULL;

	LaneMarkerLine *pLMLSel = NULL;
	int iOffsetSel = -1;
	int iYawSel = 0;
	for(int iIdx = 0; iIdx < pLMLs->get_lane_marker_line_number(); iIdx++) {
		LaneMarkerLine *pLML = pLMLs->get_lane_marker_line(iIdx);
		if(pLML == NULL)	continue;
		int iOffset = pLML->get_offset();
		int iYaw = pLML->get_yaw();
		if(iOffset < iOffsetMin)	continue;
		if(iOffset > iOffsetMax)	continue;
		if(iYaw < iYawMin)		continue;
		if(iYaw > iYawMax)		continue;
		if(pLMLSel == NULL) {
			pLMLSel = pLML;
			iOffsetSel = iOffset;
			iYawSel = iYaw;
		} else {
			if(abs(iOffsetSel - iOffsetCenter) < abs(iOffset - iOffsetCenter)) {
			} else if(abs(iOffsetSel - iOffsetCenter) > abs(iOffset - iOffsetCenter)) {
				pLMLSel = pLML;
				iOffsetSel = iOffset;
				iYawSel = iYaw;
			} else {	// get_offset is equal
				if(abs(iYawSel - iYawCenter) > abs(iYaw - iYawCenter)) {
					pLMLSel = pLML;
					iOffsetSel = iOffset;
					iYawSel = iYaw;
				} 
			}
		}
	}
	return pLMLSel;
}
LaneMarker *searchLaneMarkerForLaneBoundaryByVotes(LaneMarkers *pLMs, int iUD, int iBottomXCenter, int iBottomXMin, int iBottomXMax, int iTopZ, int iTopXCenter, int iTopXMin, int iTopXMax)
{
	if(pLMs == NULL)	return NULL;

	LaneMarker *pLMSel = NULL;
	int iBottomXSel = -1;
	int iTopXSel = -1;
	int iVotesSel = -1;
	for(int iIdx = 0; iIdx < pLMs->get_lane_marker_number(); iIdx++) {
		LaneMarker *pLM = pLMs->get_lane_marker(iIdx);
		if(pLM == NULL)	continue;
		LaneMarkerLine *pLML = pLM->get_lane_marker_line(iUD);
		if(pLML == NULL)	continue;
		int iBottomX = pLML->get_offset();
		int iTopX = pLML->get_offset() + pLML->get_yaw();
		if(iBottomX < iBottomXMin)	continue;
		if(iBottomX > iBottomXMax)	continue;
		if(iTopX < iTopXMin)	continue;
		if(iTopX > iTopXMax)	continue;
		int iVotes = pLML->get_votes();
		if(pLMSel == NULL) {
			pLMSel = pLM;
			iBottomXSel	= iBottomX;
			iTopXSel	= iTopX;
			iVotesSel = iVotes;
		} else {
			if(iVotesSel > iVotes) {
			} else if(iVotesSel < iVotes) {
				pLMSel = pLM;
				iBottomXSel	= iBottomX;
				iTopXSel	= iTopX;
				iVotesSel = iVotes;
			} else {	// get_votes are equal
				if(abs(iBottomXSel - iBottomXCenter) < abs(iBottomX - iBottomXCenter)) {
				} else if(abs(iBottomXSel - iBottomXCenter) > abs(iBottomX - iBottomXCenter)) {
					pLMSel = pLM;
					iBottomXSel = iBottomX;
					iTopXSel = iTopX;
					iVotesSel = iVotes;
				} else {	// bottom get_offset are equal
					if(abs(iTopXSel - iTopXCenter) > abs(iTopX - iTopXCenter)) {
						pLMSel = pLM;
						iBottomXSel = iBottomX;
						iTopXSel = iTopX;
						iVotesSel = iVotes;
					} 
				}
			}
		}
	}
	return pLMSel;
}



LaneMarkerLineSequence *searchLaneMarkerLineSequenceForLaneBoundary(LaneMarkerLineSequences *pLMLSs, int iUD, int iBottomXCenter, int iBottomXMin, int iBottomXMax, int iYawCenter, int iYawMin, int iYawMax)
{
	if(pLMLSs == NULL)	return NULL;

	LaneMarkerLineSequence *pLMLSSel = NULL;
	int iXSel = -1;
	int iYawSel = -1;
	for(int iIdx = 0; iIdx < pLMLSs->getLaneMarkerLineSequenceNumber(); iIdx++) {
		LaneMarkerLineSequence *pLMLS = pLMLSs->getLaneMarkerLineSequence(iIdx);
		if(pLMLS == NULL)	continue;
		if(iUD >= 0)	{	if(pLMLS->UD() != iUD)	continue;	}
		LaneMarkerLine *pLML = pLMLS->getLaneMarkerLine(CHX_NF_NEAR);
		if(pLML == NULL)	continue;
		int iX = pLML->get_offset();
		int iYaw = pLML->get_yaw();
		if(iX < iBottomXMin)	continue;
		if(iX > iBottomXMax)	continue;
		if(iYaw < iYawMin)		continue;
		if(iYaw > iYawMax)		continue;
		if(pLMLSSel == NULL) {
			pLMLSSel = pLMLS;
			iXSel = iX;
			iYawSel = iYaw;
		} else {
			if(abs(iXSel - iBottomXCenter) < abs(iX - iBottomXCenter)) {
			} else if(abs(iXSel - iBottomXCenter) > abs(iX - iBottomXCenter)) {
				pLMLSSel = pLMLS;
				iXSel = iX;
				iYawSel = iYaw;
			} else {	// get_offset is equal
				if(abs(iYawSel - iYawCenter) > abs(iYaw - iYawCenter)) {
					pLMLSSel = pLMLS;
					iXSel = iX;
					iYawSel = iYaw;
				} 
			}
		}
	}
	return pLMLSSel;
}
LaneMarkerLineSequence *searchLaneMarkerLineSequenceForLaneBoundaryByVotes(LaneMarkerLineSequences *pLMLSs, int iUD, int iBottomXCenter, int iBottomXMin, int iBottomXMax, int iYawCenter, int iYawMin, int iYawMax)
{
	if(pLMLSs == NULL)	return NULL;

	LaneMarkerLineSequence *pLMLSSel = NULL;
	int iXSel = -1;
	int iYawSel = -1;
	int iVotesSel = 0;
	for(int iIdx = 0; iIdx < pLMLSs->getLaneMarkerLineSequenceNumber(); iIdx++) {
		LaneMarkerLineSequence *pLMLS = pLMLSs->getLaneMarkerLineSequence(iIdx);
		if(pLMLS == NULL)	continue;
		if(iUD >= 0)	{	if(pLMLS->UD() != iUD)	continue;	}
		LaneMarkerLine *pLML = pLMLS->getLaneMarkerLine(CHX_NF_NEAR);
		if(pLML == NULL)	continue;
		int iX = pLML->get_offset();
		int iYaw = pLML->get_yaw();
		if(iX < iBottomXMin)	continue;
		if(iX > iBottomXMax)	continue;
		if(iYaw < iYawMin)		continue;
		if(iYaw > iYawMax)		continue;
		int iVotes = pLMLS->Near()->get_votes() + pLMLS->Far()->get_votes();
		if(pLMLSSel == NULL) {
			pLMLSSel = pLMLS;
			iXSel = iX;
			iYawSel = iYaw;
			iVotesSel = iVotes;
		} else {
			if(iVotesSel < iVotes) {
				pLMLSSel = pLMLS;
				iXSel = iX;
				iYawSel = iYaw;
				iVotesSel = iVotes;
			} else if(iVotesSel > iVotes) {
			} else {
				if(abs(iXSel - iBottomXCenter) < abs(iX - iBottomXCenter)) {
				} else if(abs(iXSel - iBottomXCenter) > abs(iX - iBottomXCenter)) {
					pLMLSSel = pLMLS;
					iXSel = iX;
					iYawSel = iYaw;
					iVotesSel = iVotes;
				} else {	// get_offset is equal
					if(abs(iYawSel - iYawCenter) > abs(iYaw - iYawCenter)) {
						pLMLSSel = pLMLS;
						iXSel = iX;
						iYawSel = iYaw;
						iVotesSel = iVotes;
					}
				} 
			}
		}
	}
	return pLMLSSel;
}
LaneMarkerLineSequence *searchLaneMarkerLineSequenceForLaneBoundary2(LaneMarkerLineSequences *pLMLSs, int iUD,
	int iNearBottomXCenter, int iNearBottomXMin, int iNearBottomXMax,
	int iNearTopXCenter, int iNearTopXMin, int iNearTopXMax
	)
{
	if(pLMLSs == NULL)	return NULL;

	LaneMarkerLineSequence *pLMLSSel = NULL;
	int iNearBottomXSel = -1;
	int iNearTopXSel = -1;
	for(int iIdx = 0; iIdx < pLMLSs->getLaneMarkerLineSequenceNumber(); iIdx++) {
		LaneMarkerLineSequence *pLMLS = pLMLSs->getLaneMarkerLineSequence(iIdx);
		if(pLMLS == NULL)	continue;
		if(iUD >= 0)	{	if(pLMLS->UD() != iUD)	continue;	}
		LaneMarkerLine *pLML = pLMLS->getLaneMarkerLine(CHX_NF_NEAR);
		if(pLML == NULL)	continue;
		int iNearBottomX = pLML->get_offset();
		if(iNearBottomX < iNearBottomXMin)	continue;
		if(iNearBottomX > iNearBottomXMax)	continue;
		int iNearTopX = pLML->get_offset() + pLML->get_yaw();
		if(iNearTopX < iNearTopXMin)	continue;
		if(iNearTopX > iNearTopXMax)	continue;
		if(pLMLSSel == NULL) {
			pLMLSSel = pLMLS;
			iNearBottomXSel	= iNearBottomX;
			iNearTopXSel	= iNearTopX;
		} else {
			if(abs(iNearBottomXSel - iNearBottomXCenter) < abs(iNearBottomX - iNearBottomXCenter)) {
			} else if(abs(iNearBottomXSel - iNearBottomXCenter) > abs(iNearBottomX - iNearBottomXCenter)) {
				pLMLSSel = pLMLS;
				iNearBottomXSel	= iNearBottomX;
				iNearTopXSel	= iNearTopX;
			} else {	// get_offset is equal
				if(abs(iNearTopXSel - iNearTopXCenter) < abs(iNearTopX - iNearTopXCenter)) {
					pLMLSSel = pLMLS;
					iNearBottomXSel	= iNearBottomX;
					iNearTopXSel	= iNearTopX;
				} 
			}
		}
	}
	return pLMLSSel;
}
LaneMarkerLineSequence *searchLaneMarkerLineSequenceForLaneBoundaryByVotes2(LaneMarkerLineSequences *pLMLSs, int iUD,
	int iNearBottomXCenter, int iNearBottomXMin, int iNearBottomXMax,
	int iNearTopXCenter, int iNearTopXMin, int iNearTopXMax
	)
{
	if(pLMLSs == NULL)	return NULL;

	LaneMarkerLineSequence *pLMLSSel = NULL;
	int iNearBottomXSel = -1;
	int iNearTopXSel = -1;
	int iVotesSel = 0;
	for(int iIdx = 0; iIdx < pLMLSs->getLaneMarkerLineSequenceNumber(); iIdx++) {
		LaneMarkerLineSequence *pLMLS = pLMLSs->getLaneMarkerLineSequence(iIdx);
		if(pLMLS == NULL)	continue;
		if(iUD >= 0)	{	if(pLMLS->UD() != iUD)	continue;	}
		LaneMarkerLine *pLML = pLMLS->getLaneMarkerLine(CHX_NF_NEAR);
		if(pLML == NULL)	continue;
		int iNearBottomX = pLML->get_offset();
		if(iNearBottomX < iNearBottomXMin)	continue;
		if(iNearBottomX > iNearBottomXMax)	continue;
		int iNearTopX = pLML->get_offset() + pLML->get_yaw();
		if(iNearTopX < iNearTopXMin)	continue;
		if(iNearTopX > iNearTopXMax)	continue;
		int iVotes = pLMLS->Near()->get_votes() + pLMLS->Far()->get_votes();
		if(pLMLSSel == NULL) {
			pLMLSSel = pLMLS;
			iNearBottomXSel	= iNearBottomX;
			iNearTopXSel	= iNearTopX;
			iVotesSel = iVotes;
		} else {
			if(iVotesSel < iVotes) {
				pLMLSSel = pLMLS;
				iNearBottomXSel	= iNearBottomX;
				iNearTopXSel	= iNearTopX;
				iVotesSel = iVotes;
			} else if(iVotesSel > iVotes) {
			} else {
				if(abs(iNearBottomXSel - iNearBottomXCenter) < abs(iNearBottomX - iNearBottomXCenter)) {
				} else if(abs(iNearBottomXSel - iNearBottomXCenter) > abs(iNearBottomX - iNearBottomXCenter)) {
					pLMLSSel = pLMLS;
					iNearBottomXSel	= iNearBottomX;
					iNearTopXSel	= iNearTopX;
					iVotesSel = iVotes;
				} else {	// get_offset is equal
					if(abs(iNearTopXSel - iNearTopXCenter) < abs(iNearTopX - iNearTopXCenter)) {
						pLMLSSel = pLMLS;
						iNearBottomXSel	= iNearBottomX;
						iNearTopXSel	= iNearTopX;
						iVotesSel = iVotes;
					}
				} 
			}
		}
	}
	return pLMLSSel;
}
LaneMarkerLine *searchLaneMarkerLineForLaneBoundary2(LaneMarkerLines *pLMLs,
	int iBottomXCenter, int iBottomXMin, int iBottomXMax,
	int iTopXCenter, int iTopXMin, int iTopXMax
	)
{
	if(pLMLs == NULL)	return NULL;

	LaneMarkerLine *pLMLSel = NULL;
	int iBottomXSel = -1;
	int iTopXSel = -1;
	for(int iIdx = 0; iIdx < pLMLs->get_lane_marker_line_number(); iIdx++) {
		LaneMarkerLine *pLML = pLMLs->get_lane_marker_line(iIdx);
		if(pLML == NULL)	continue;
		int iBottomX = pLML->get_offset();
		if(iBottomX < iBottomXMin)	continue;
		if(iBottomX > iBottomXMax)	continue;
		int iTopX = pLML->get_offset() + pLML->get_yaw();
		if(iTopX < iTopXMin)	continue;
		if(iTopX > iTopXMax)	continue;
		if(pLMLSel == NULL) {
			pLMLSel = pLML;
			iBottomXSel	= iBottomX;
			iTopXSel	= iTopX;
		} else {
			if(abs(iBottomXSel - iBottomXCenter) < abs(iBottomX - iBottomXCenter)) {
			} else if(abs(iBottomXSel - iBottomXCenter) > abs(iBottomX - iBottomXCenter)) {
				pLMLSel = pLML;
				iBottomXSel	= iBottomX;
				iTopXSel	= iTopX;
			} else {	// get_offset is equal
				if(abs(iTopXSel - iTopXCenter) > abs(iTopX - iTopXCenter)) {
					pLMLSel = pLML;
					iBottomXSel	= iBottomX;
					iTopXSel	= iTopX;
				} 
			}
		}
	}
	return pLMLSel;
}

double g_dbOffsetSearchMarginSequence = OFFSET_SEARCH_MARGIN_SEQUENCE;				//// 
double g_dbYawSearchMarginSequence = YAW_SEARCH_MARGIN_SEQUENCE;	// //
double g_dbOffsetSearchMarginTrack = OFFSET_SEARCH_MARGIN_TRACK;	// //
double g_dbYawSearchMarginTrack = YAW_SEARCH_MARGIN_TRACK;
double g_dbOffsetSearchMarginForComplexLaneBoundary = OFFSET_SEARCH_MARGIN_TRACK * 2;		// //

///////////////////////
LaneMarker *LaneArea::searchLaneMarkerForLaneBoundaryByInitialParameter(int iNF, int iLR)								// 01
{
	LaneRegion *pRegion = get_region(iNF);
	LaneParameterOneSide *pLPOneSide = getLaneParameterOneSide(iLR);
	if(pRegion == NULL)	return NULL;
	int iUD = iLR == CHX_LR_LEFT ? CHX_UD_DOWN : CHX_UD_UP;	//// 
	int iK = iLR == CHX_LR_LEFT ? -1 : 1;			// ////
	LaneSide *pSide = pRegion->get_side(iLR);	// ////
	if(pSide == NULL)	return NULL;
	double dBottomXCenter = pLPOneSide->ParamInit(CHX_LPID_OFFSET);		// //
	double dBottomXMin = dBottomXCenter;	// //
	double dBottomXMax = dBottomXCenter;	// 
	if(iLR == CHX_LR_LEFT) {
		dBottomXMin -= OUTSIDE_OF_LANEBOUNDARY_FOR_LANE_BOUNDARY;
		dBottomXMax = -MARGIN_FOR_SEARCH_LANEMARKER_WITH_INITIAL_PARAMETER;	// 20111121//
	} else {
		dBottomXMin = MARGIN_FOR_SEARCH_LANEMARKER_WITH_INITIAL_PARAMETER;	// 20111121//
		dBottomXMax += OUTSIDE_OF_LANEBOUNDARY_FOR_LANE_BOUNDARY;
	}
	double dYawCenter = 0.0;									// //
	double dYawMin = dYawCenter - DB_YAW_MAX;				// //
	double dYawMax = dYawCenter + DB_YAW_MAX;				// //
	double dBottomZ = pRegion->get_bottom();					// //
	LaneMarkers *pLMs = pSide->get_lane_markers();			// //
	int iOffsetCenter = pSide->transform_Xvehicle_to_Jroad(dBottomXCenter);	////
	int iOffsetMin = pSide->transform_Xvehicle_to_Jroad(dBottomXMin);	////
	int iOffsetMax = pSide->transform_Xvehicle_to_Jroad(dBottomXMax);	////
	int iYawCenter = 0;									// 
	int iYawMargin = (int)((tan(DB_YAW_MAX) * pSide->get_height() / pSide->get_width()) * pSide->get_road_image_width());
	int iYawMin = iYawCenter - iYawMargin;				// //
	int iYawMax = iYawCenter + iYawMargin;				// //
	LaneMarker *pLM = searchLaneMarkerForLaneBoundaryByVotes(pLMs, iUD, iOffsetCenter, iOffsetMin, iOffsetMax, iYawCenter, iYawMin, iYawMax);

	if(pLM != NULL) {
		LaneMarker *pNewLM = new LaneMarker(pLM);	// //
		pSide->setLaneBoundary(pNewLM);	// //
		pSide->setFoundNowFlag();		////
		pSide->LaneBoundaryType(0, CHX_LBT_LANEMARKER);		// //
	}

	return pLM;
}

LaneMarkerLineSequence *LaneArea::searchLaneMarkerLineSequenceForLaneBoundaryByInitialParameter(int iLR)				// 02
{
	LaneRegion *pNearRegion = get_region(CHX_NF_NEAR);	// //
	if(pNearRegion == NULL)	return false;
	double dZNearBottom = pNearRegion->get_bottom();
	LaneRegion *pFarRegion = get_region(CHX_NF_FAR);	// //
	if(pFarRegion == NULL)	return false;
	LaneSide *pNearSide = pNearRegion->get_side(iLR);	// 
	LaneSide *pFarSide = pFarRegion->get_side(iLR);		// //	
	LaneParameterOneSide *pLPOneSide = getLaneParameterOneSide(iLR);

	int iK = iLR == CHX_LR_LEFT ? -1 : 1;			// // -1, // 1

	double dNearBottomXCenter = pLPOneSide->ParamInit(CHX_LPID_OFFSET);		// //
	double dNearBottomXMin = dNearBottomXCenter + DB_OFFSET_MIN;	// 
	double dNearBottomXMax = dNearBottomXCenter + DB_OFFSET_MAX;	// 
	double dNearYawCenter = 0.0;									// 
	double dNearYawMin = dNearYawCenter - DB_YAW_MAX;				// //
	double dNearYawMax = dNearYawCenter + DB_YAW_MAX;				// //
	double dNearBottomZ = get_region(CHX_NF_NEAR)->get_bottom();					// ////

	LaneMarkerLineSequences *pLMLSs = getLaneMarkerLineSequences(iLR);
	int iOffsetCenter = pNearSide->transform_Xvehicle_to_Jroad(dNearBottomXCenter);	// //(lateral)
	int iOffsetMin = pNearSide->transform_Xvehicle_to_Jroad(dNearBottomXMin);	// //(lateral)
	int iOffsetMax = pNearSide->transform_Xvehicle_to_Jroad(dNearBottomXMax);	// //(lateral)
	int iYawCenter = 0;									// 
	int iYawMargin = (int)((tan(DB_YAW_MAX) * pNearSide->get_height() / pNearSide->get_width()) * pNearSide->get_road_image_width());
	int iYawMin = iYawCenter - iYawMargin;				// //
	int iYawMax = iYawCenter + iYawMargin;				// //
	LaneMarkerLineSequence *pLMLS = searchLaneMarkerLineSequenceForLaneBoundaryByVotes(pLMLSs, -1, iOffsetCenter, iOffsetMin, iOffsetMax, iYawCenter, iYawMin, iYawMax);	// 20110831



	if(pLMLS != NULL) {		////			// (a)-(a)-EDGE-EDGE
		int iEdgeDir = pLMLS->UD() == CHX_UD_UP ? CHX_LBT_UP : CHX_LBT_DOWN;
		LaneMarker *pNewNearLM = new LaneMarker(pLMLS->Near(), pLMLS->Near());	// //
		pNearSide->setLaneBoundary(pNewNearLM);	////
		pNearSide->setFoundNowFlag();		// //
		pNearSide->LaneBoundaryType(0, CHX_LBT_LANEMARKERLINE);
		pNearSide->LaneBoundaryType(1, iEdgeDir);
		LaneMarker *pNewFarLM = new LaneMarker(pLMLS->Far(), pLMLS->Far());	// //
		pFarSide->setLaneBoundary(pNewFarLM);	// ////
		pFarSide->setFoundNowFlag();		// //
		pFarSide->LaneBoundaryType(0, CHX_LBT_LANEMARKERLINE);
		pFarSide->LaneBoundaryType(1, iEdgeDir);
	}

	return pLMLS;
}
LaneMarker *LaneArea::searchLaneMarkerForLaneBoundaryByNearerRegion(int iNF, int iLR)									// 03
{
	LaneRegion *pRegion = get_region(iNF);
	if(pRegion == NULL)	return NULL;
	LaneRegion *pNearerRegion = get_region(iNF - 1);
	if(pNearerRegion == NULL)	return NULL;

	LaneMarker *pNearerLM = pNearerRegion->get_side(iLR)->getLaneBoundary();
	if(pNearerLM == NULL)	return NULL;

	int iUD = iLR == CHX_LR_LEFT ? CHX_UD_DOWN : CHX_UD_UP;	// //// // 
	LaneMarkerLine *pNearerLML = pNearerRegion->get_side(iLR)->getLaneBoundary()->get_lane_marker_line(iUD);
	if(pNearerLML == NULL)	return NULL;

	LaneSide *pSide = pRegion->get_side(iLR);		// //	
	if(pSide == NULL)	return NULL;
	LaneMarkers *pLMs = pSide->get_lane_markers();

	int iOffsetCenter = pNearerLML->get_offset() + pNearerLML->get_yaw();	// //(lateral)
	int iOffsetMargin = pSide->transform_Xvehicle_to_Jroad(g_dbOffsetSearchMarginTrack) - pSide->transform_Xvehicle_to_Jroad(0);
	int iOffsetMin = iOffsetCenter - iOffsetMargin;	// //(lateral)
	int iOffsetMax = iOffsetCenter + iOffsetMargin;	// //(lateral)
	int iYawCenter = pNearerLML->get_yaw();									// 
	int iYawMargin = (int)((tan(DB_YAW_MAX) * pSide->get_height() / pSide->get_width()) * pSide->get_road_image_width());
	int iYawMin = iYawCenter - iYawMargin;				// //
	int iYawMax = iYawCenter + iYawMargin;				// //
	LaneMarker *pLM = searchLaneMarkerForLaneBoundary(pLMs, iUD, iOffsetCenter, iOffsetMin, iOffsetMax, iYawCenter, iYawMin, iYawMax);

	if(pLM != NULL) {	// //				// (a)-(a)-LM-LM
		LaneMarker *pNewLM = new LaneMarker(pLM);	// 
		pSide->setLaneBoundary(pNewLM);	// ////
		pSide->setFoundNowFlag();		// //
		pSide->LaneBoundaryType(0, CHX_LBT_LANEMARKER);

	}

	return pLM;
}
LaneMarker *LaneArea::searchLaneMarkerForLaneBoundaryByPreviousFrame(int iNF, int iLR)								// 04
{
	LaneRegion *pRegion = get_region(iNF);

	LaneMarker *pLMPre = NULL;
	for(int iNFtmp = iNF; iNFtmp >= 0; iNFtmp--) {
		pLMPre = get_region(iNFtmp)->get_side(iLR)->getLaneBoundary();
		if(pLMPre != NULL)	break;
	}
	if(pLMPre == NULL)	return NULL;

	int iUD = iLR == CHX_LR_LEFT ? CHX_UD_DOWN : CHX_UD_UP;	// /////// 


	LaneSide *pSide = pRegion->get_side(iLR);	// 
	if(pSide == NULL)	return NULL;

	LaneMarkers *pLMs = pSide->get_lane_markers();
	int iOffsetCenter = pLMPre->get_lane_marker_line(iUD)->get_offset();	// //(lateral)
	int iOffsetMargin = pSide->transform_Xvehicle_to_Jroad(g_dbOffsetSearchMarginTrack) - pSide->transform_Xvehicle_to_Jroad(0);
	int iOffsetMin = iOffsetCenter - iOffsetMargin;	// //(lateral)
	int iOffsetMax = iOffsetCenter + iOffsetMargin;	// //(lateral)
	int iYawCenter = pLMPre->get_lane_marker_line(iUD)->get_yaw();
	int iYawMargin = (int)((tan(DB_YAW_MAX) * pSide->get_height() / pSide->get_width()) * pSide->get_road_image_width());
	int iYawMin = iYawCenter - iYawMargin;				// //
	int iYawMax = iYawCenter + iYawMargin;				// //
	LaneMarker *pLM = searchLaneMarkerForLaneBoundary(pLMs, iUD, iOffsetCenter, iOffsetMin, iOffsetMax, iYawCenter, iYawMin, iYawMax);

	if(pLM != NULL ) {
		LaneMarker *pNewLM = new LaneMarker(pLM);	// 
		pSide->setLaneBoundary(pNewLM);	//////
		pSide->setFoundNowFlag();		// //
		pSide->LaneBoundaryType(0, CHX_LBT_LANEMARKER);
	}
	return pLM;
}
LaneMarker *LaneArea::searchLaneMarkerForLaneBoundaryByPreviousFurtherFrame(int iNF, int iLR)								// 04
{
	LaneRegion *pRegion = get_region(iNF);

	LaneMarker *pLMPre = NULL;
	for(int iNFtmp = iNF; iNFtmp < CHX_NF_NUM; iNFtmp++) {
		pLMPre = get_region(iNFtmp)->get_side(iLR)->getLaneBoundary();
		if(pLMPre != NULL)	break;
	}
	if(pLMPre == NULL)	return NULL;

	int iUD = iLR == CHX_LR_LEFT ? CHX_UD_DOWN : CHX_UD_UP;	// ////, // 
	LaneMarkerLine *pLMLPre = pLMPre->get_lane_marker_line(iUD);
	if(pLMLPre == NULL)	return NULL;


	LaneSide *pSide = pRegion->get_side(iLR);	// 
	if(pSide == NULL)	return NULL;

	LaneMarkers *pLMs = pSide->get_lane_markers();
	int iOffsetCenter = pLMLPre->get_offset() - pLMLPre->get_yaw();
	int iOffsetMargin = pSide->transform_Xvehicle_to_Jroad(g_dbOffsetSearchMarginTrack) - pSide->transform_Xvehicle_to_Jroad(0);
	int iOffsetMin = iOffsetCenter - iOffsetMargin;	// //(lateral)
	int iOffsetMax = iOffsetCenter + iOffsetMargin;	// //(lateral)
	int iYawCenter = pLMLPre->get_yaw();									// 
	int iYawMargin = (int)((tan(DB_YAW_MAX) * pSide->get_height() / pSide->get_width()) * pSide->get_road_image_width());
	int iYawMin = iYawCenter - iYawMargin;				// //
	int iYawMax = iYawCenter + iYawMargin;				// //
	LaneMarker *pLM = searchLaneMarkerForLaneBoundary(pLMs, iUD, iOffsetCenter, iOffsetMin, iOffsetMax, iYawCenter, iYawMin, iYawMax);

	if(pLM != NULL ) {
		LaneMarker *pNewLM = new LaneMarker(pLM);	// 
		pSide->setLaneBoundary(pNewLM);	////
		pSide->setFoundNowFlag();		// //
		pSide->LaneBoundaryType(0, CHX_LBT_LANEMARKER);
	}
	return pLM;
}
LaneMarkerLineSequence *LaneArea::searchLaneMarkerLineSequenceForLaneBoundaryByPreviousFrame(int iLR)					// 05
{
	int iUD = iLR == CHX_LR_LEFT ? CHX_UD_DOWN : CHX_UD_UP;	// ////, // 

	LaneRegion *pNearRegion = get_region(CHX_NF_NEAR);	// 
	if(pNearRegion == NULL)	return NULL;
	LaneRegion *pFarRegion = get_region(CHX_NF_FAR);	// ////
	if(pFarRegion == NULL)	return NULL;
	LaneSide *pNearSide = pNearRegion->get_side(iLR);	// 
	if(pNearSide == NULL)	return NULL;
	LaneSide *pFarSide = pFarRegion->get_side(iLR);		// //	
	if(pFarSide == NULL)	return NULL;

	LaneMarker *pLMPre = pNearSide->getLaneBoundary();
	if(pLMPre == NULL)	return NULL;
	LaneMarkerLine *pLMLPre = pLMPre->get_lane_marker_line(iUD);
	if(pLMLPre == NULL)	return NULL;

	LaneMarkerLineSequences *pLMLSs = getLaneMarkerLineSequences(iLR);
	int iOffsetCenter = pLMLPre->get_offset();	// //(lateral)
	int iOffsetMargin = pNearSide->transform_Xvehicle_to_Jroad(g_dbOffsetSearchMarginTrack) - pNearSide->transform_Xvehicle_to_Jroad(0);
	int iOffsetMin = iOffsetCenter - iOffsetMargin;	// //(lateral)
	int iOffsetMax = iOffsetCenter + iOffsetMargin;	// //(lateral)
	int iYawCenter = pLMLPre->get_yaw();
	int iYawMargin = (int)((tan(DB_YAW_MAX) * pNearSide->get_height() / pNearSide->get_width()) * pNearSide->get_road_image_width());
	int iYawMin = iYawCenter - iYawMargin;				// //
	int iYawMax = iYawCenter + iYawMargin;				// //
	LaneMarkerLineSequence *pLMLS = searchLaneMarkerLineSequenceForLaneBoundary(pLMLSs, iUD, iOffsetCenter, iOffsetMin, iOffsetMax, iYawCenter, iYawMin, iYawMax);

	if(pLMLS != NULL) {	////	// (a)-(b)-EDGE-EDGE
		LaneMarker *pNewNearLM = new LaneMarker(pLMLS->Near(), pLMLS->Near());	// //
		pNearSide->setLaneBoundary(pNewNearLM);	////
		pNearSide->setFoundNowFlag();		// //
		pNearSide->LaneBoundaryType(0, CHX_LBT_LANEMARKERLINE);
		LaneMarker *pNewFarLM = new LaneMarker(pLMLS->Far(), pLMLS->Far());	// //
		pFarSide->setLaneBoundary(pNewFarLM);	////
		pFarSide->setFoundNowFlag();		// //
		pFarSide->LaneBoundaryType(0, CHX_LBT_LANEMARKERLINE);
	}
	return pLMLS;
}
LaneMarker *LaneArea::searchLaneMarkerForLaneBoundaryByParameter(int iNF, int iLR)									// 06
{
	int iUD = iLR == CHX_LR_LEFT ? CHX_UD_DOWN : CHX_UD_UP;	// ////, // 
	int iK = iLR == CHX_LR_LEFT ? -1 : 1;			// // -1, // 1

	LaneParameterOneSide *pLaneParameterOneSide = getLaneParameterOneSide(iLR);
	if(pLaneParameterOneSide == NULL)	return NULL;
	LaneRegion *pRegion = get_region(iNF);
	if(pRegion == NULL)	return NULL;
	LaneSide *pSide = pRegion->get_side(iLR);
	double dBottomZ = pRegion->get_bottom();
	double dBottomX = pLaneParameterOneSide->LaneBoundaryPositionOnRoad(iK, dBottomZ);
	double dBottomXCenter = dBottomX;
	double dBottomXMin = dBottomXCenter - g_dbOffsetSearchMarginTrack;
	double dBottomXMax = dBottomXCenter + g_dbOffsetSearchMarginTrack;

	double dTopZ = pRegion->get_top();
	double dTopX = 0;
	if(pLaneParameterOneSide->Available() == true)	{
		dTopX = pLaneParameterOneSide->LaneBoundaryPositionOnRoad(iK, dTopZ);
	} else {
		return NULL;
	}
	double dTopXCenter = dTopX;
	double dTopXMin = dTopXCenter - g_dbOffsetSearchMarginTrack;
	double dTopXMax = dTopXCenter + g_dbOffsetSearchMarginTrack;
	LaneMarkers *pLMs = pSide->get_lane_markers();
	int iBottomOffsetCenter = pSide->transform_Xvehicle_to_Jroad(dBottomXCenter);	// //(lateral)
	int iBottomOffsetMin = pSide->transform_Xvehicle_to_Jroad(dBottomXMin);	// //(lateral)
	int iBottomOffsetMax = pSide->transform_Xvehicle_to_Jroad(dBottomXMax);	// //(lateral)
	int iTopOffsetCenter = pSide->transform_Xvehicle_to_Jroad(dTopXCenter);	// //(lateral)
	int iTopOffsetMin = pSide->transform_Xvehicle_to_Jroad(dTopXMin);	// //(lateral)
	int iTopOffsetMax = pSide->transform_Xvehicle_to_Jroad(dTopXMax);	// //(lateral)
	int iYawMin = iTopOffsetMin - iBottomOffsetMax;
	int iYawMax = iTopOffsetMax - iBottomOffsetMin;
	int iYawCenter = (iBottomOffsetMin + iBottomOffsetMax) / 2;
	LaneMarker *pLM = searchLaneMarkerForLaneBoundaryByVotes(pLMs, iUD, iBottomOffsetCenter, iBottomOffsetMin, iBottomOffsetMax, iYawCenter, iYawMin, iYawMax);	// 20110920

	if(pLM != NULL) {	// ////	// (b)-(a)-LM-
		LaneMarker *pNewLM = new LaneMarker(pLM);	// 
		pSide->setLaneBoundary(pNewLM);	////
		pSide->setFoundNowFlag();		// //
		pSide->LaneBoundaryType(0, CHX_LBT_LANEMARKER);
	}
	return pLM;

}

bool LaneArea::detectLaneMarkerLineSequences(CamParam *pCamParam)
{
	if(pCamParam == NULL)	return false;
	for(int iLR = CHX_LR_LEFT; iLR < CHX_LR_NUM; iLR++) {	// ////////
		LaneMarkerLineSequences *pLMLSs = new LaneMarkerLineSequences();	////////////
		if(pLMLSs == NULL)	continue;

		for(int iUD = CHX_UD_UP; iUD < CHX_UD_NUM; iUD++) {	////////
			LaneRegion *pNearRegion = get_region(CHX_NF_NEAR);
			if(pNearRegion == NULL)	continue;
			LaneSide *pNearSide = pNearRegion->get_side(iLR);
			if(pNearSide == NULL)	continue;
			LaneMarkerLines *pNearLMLs = pNearSide->get_lane_marker_lines(iUD);	// ///////
			if(pNearLMLs == NULL)	continue;
			LaneRegion *pFarRegion = get_region(CHX_NF_FAR);
			if(pFarRegion == NULL)continue;
			LaneSide *pFarSide = pFarRegion->get_side(iLR);
			if(pFarSide == NULL)	continue;
			LaneMarkerLines *pFarLMLs = pFarSide->get_lane_marker_lines(iUD);		// ////////
			if(pFarLMLs == NULL)	continue;
			for(int iIdxN = 0; iIdxN < pNearLMLs->get_lane_marker_line_number(); iIdxN++) {	////////
				LaneMarkerLine *pNearLML = pNearLMLs->get_lane_marker_line(iIdxN);
				if(pNearLML == NULL)	continue;
				if(abs(pNearLML->get_yaw())  > MAXIMUM_YAW_OF_LANEMARKERLINESEQUENCE)	continue;	// ////////
				if(	(pNearLML->get_offset() > pNearSide->get_bb(iUD)->get_number_of_offset() * MINIMUM_OFFSET_FOR_LANEMARKERLINESEQUENCE_INHIBIT)	// (//·)//
					&&	(pNearLML->get_offset() < pNearSide->get_bb(iUD)->get_number_of_offset() * MAXIMUM_OFFSET_FOR_LANEMARKERLINESEQUENCE_INHIBIT)
					){
						continue;
				}
				int iNearBottomJ = pNearLML->get_offset();				// ////
				int iNearTopJ = iNearBottomJ + pNearLML->get_yaw();		// ////
				double dNearBottomX = pNearRegion->get_left() + (pNearRegion->get_right() -pNearRegion->get_left()) / pNearSide->get_road_image_width()* (iNearBottomJ + 0.5);	// ?·?(//)
				double dNearBottomY = pNearRegion->get_bottom();																									//////
				double dNearTopY = pNearRegion->get_top();																											// //////
				double dNearTopX = pNearRegion->get_left() + (pNearRegion->get_right() -pNearRegion->get_left()) / pNearSide->get_road_image_width() * (iNearTopJ + 0.5);			// ?h·?(//)

				double dMaximumHorizontalDiffAtRegionBoundary = MAXIMUM_HORIZONTAL_DIFF_AT_REGION_BOUNDARY;	// //
				double dBottomX = dNearTopX;					////
				double dBottomXMin = dBottomX - dMaximumHorizontalDiffAtRegionBoundary;//50;//100.;		// ////
				double dBottomXMax = dBottomX + dMaximumHorizontalDiffAtRegionBoundary;//50;//100.;		// ////

				int iSearchOffsetJMin = pNearSide->transform_Xvehicle_to_Jroad(dBottomXMin);	//////
				int iSearchOffsetJMax = pNearSide->transform_Xvehicle_to_Jroad(dBottomXMax);	//////

				int iMaximumYawDiffAtRegionBoundary = MAXIMUM_YAW_DIFF_AT_REGION_BOUNDARY; // //
				int iSearchYawMin = pNearLML->get_yaw() - iMaximumYawDiffAtRegionBoundary;//10;//5;
				int iSearchYawMax = pNearLML->get_yaw() + iMaximumYawDiffAtRegionBoundary;//10;//5;

				for(int iIdxF = 0; iIdxF < pFarLMLs->get_lane_marker_line_number(); iIdxF++) {
					LaneMarkerLine *pFarLML = pFarLMLs->get_lane_marker_line(iIdxF);
					if(pFarLML == NULL)	continue;
					if(pFarLML->get_offset()	< iSearchOffsetJMin)	continue;	// //
					if(pFarLML->get_offset()	> iSearchOffsetJMax)	continue;	// //
					if(pFarLML->get_yaw()		< iSearchYawMin)		continue;	// //
					if(pFarLML->get_yaw()		> iSearchYawMax)		continue;	////
					LaneMarkerLineSequence *pNewLMLS = new LaneMarkerLineSequence();	// ////
					LaneMarkerLine *pNewNearLML = new LaneMarkerLine(pNearLML);			// 
					LaneMarkerLine *pNewFarLML = new LaneMarkerLine(pFarLML);			////
					pNewLMLS->Near(pNewNearLML);										// ////
					pNewLMLS->Far(pNewFarLML);											// ///////
					pNewLMLS->UD(iUD);
					pLMLSs->addLaneMarkerLineSequence(pNewLMLS);					// //
				}
			}
		}
		setLaneMarkerLineSequences(iLR, pLMLSs);	// ////////
	}
	return true;
}
double LaneArea::calcYawDiffNearAndFar(int iLR, int iNF0, int iNF1)
{
	LaneParameterOneSide *pLaneParameterOneSide = getLaneParameterOneSide(iLR);
	if(pLaneParameterOneSide == NULL)	return NULL;
	LaneRegion *pRegion0 = get_region(iNF0);
	double dBottomZ0 = pRegion0->get_bottom();
	double dBottomX0 = pLaneParameterOneSide->LaneBoundaryPositionOnRoad(0, dBottomZ0);
	double dTopZ0 = pRegion0->get_top();
	double dTopX0 = pLaneParameterOneSide->LaneBoundaryPositionOnRoad(0, dTopZ0);
	double dYaw0 = atan((dTopX0 - dBottomX0) / (dTopZ0 - dBottomZ0));
	LaneRegion *pRegion1 = get_region(iNF1);
	double dBottomZ1 = pRegion1->get_bottom();
	double dBottomX1 = pLaneParameterOneSide->LaneBoundaryPositionOnRoad(0, dBottomZ1);
	double dTopZ1 = pRegion1->get_top();
	double dTopX1 = pLaneParameterOneSide->LaneBoundaryPositionOnRoad(0, dTopZ1);
	double dYaw1 = atan((dTopX1 - dBottomX1) / (dTopZ1 - dBottomZ1));

	double dYawDiff = dYaw1 - dYaw0;
	return dYawDiff;
}
LaneMarker *LaneArea::searchLaneMarkerForLaneBoundaryByNearerRegionAndParameter(int iNF, int iLR)						// 07
{
	LaneRegion *pRegion = get_region(iNF);
	if(pRegion == NULL)	return NULL;
	LaneRegion *pNearerRegion = get_region(iNF - 1);
	if(pNearerRegion == NULL)	return NULL;

	LaneMarker *pNearerLM = pNearerRegion->get_side(iLR)->getLaneBoundary();
	if(pNearerLM == NULL)	return NULL;

	int iUD = iLR == CHX_LR_LEFT ? CHX_UD_DOWN : CHX_UD_UP;	// ////, // 
	int iK = iLR == CHX_LR_LEFT ? -1 : 1;			// // -1, // 1


	LaneSide *pSide = pRegion->get_side(iLR);		// //	
	if(pSide == NULL)	return NULL;
	LaneMarkers *pLMs = pSide->get_lane_markers();

	int iOffsetCenter = pNearerLM->get_lane_marker_line(iUD)->get_offset() + pNearerLM->get_lane_marker_line(iUD)->get_yaw();
	int iOffsetMin = iOffsetCenter - (int)(g_dbOffsetSearchMarginSequence / get_region(iNF)->get_side(iNF)->get_width() * get_region(iNF)->get_side(iNF)->get_road_image_width());
	int iOffsetMax = iOffsetCenter + (int)(g_dbOffsetSearchMarginSequence / get_region(iNF)->get_side(iNF)->get_width() * get_region(iNF)->get_side(iNF)->get_road_image_width());
	int iYawCenter = pNearerLM->get_lane_marker_line(iUD)->get_yaw();
	double dYawDiff = calcYawDiffNearAndFar(iLR, CHX_NF_NEAR, CHX_NF_FAR);
	int iYawDiff = (int)((tan(dYawDiff) *pSide->get_height() / pSide->get_width()) * pSide->get_road_image_width());
	iYawCenter += iYawDiff;
	int iYawMargin = (int)((tan(g_dbYawSearchMarginSequence) * pSide->get_height() / pSide->get_width()) * pSide->get_road_image_width());
	int iYawMin = iYawCenter - iYawMargin;				// //
	int iYawMax = iYawCenter + iYawMargin;				// //
	LaneMarker *pLM = searchLaneMarkerForLaneBoundary(pLMs, iUD, iOffsetCenter, iOffsetMin, iOffsetMax, iYawCenter, iYawMin, iYawMax);

	if(pLM != NULL)	{	// //				// (a)-(a)-LM-LM
		LaneMarker *pNewLM = new LaneMarker(pLM);	// 
		pSide->setLaneBoundary(pNewLM);	////
		pSide->setFoundNowFlag();		// //
		pSide->LaneBoundaryType(0, CHX_LBT_LANEMARKER);
	}

	return pLM;
}
LaneMarkerLineSequence *LaneArea::searchLaneMarkerLineSequenceForLaneBoundaryByParameter(int iLR)						// 08
{
	LaneRegion *pNearRegion = get_region(CHX_NF_NEAR);
	LaneSide *pNearSide = pNearRegion->get_side(iLR);
	LaneRegion *pFarRegion = get_region(CHX_NF_FAR);
	LaneSide *pFarSide = pFarRegion->get_side(iLR);
	int iEdgeDir = pNearSide->LaneBoundaryType(1);
	int iUD = iEdgeDir == CHX_LBT_UP ? CHX_UD_UP : CHX_UD_DOWN;
	int iK = iLR == CHX_LR_LEFT ? -1 : 1;			// // -1, // 1

	LaneParameterOneSide *pLaneParameterOneSide = getLaneParameterOneSide(iLR);
	if(pLaneParameterOneSide == NULL)	return NULL;
	double dNearBottomZ = pNearRegion->get_bottom();
	double dNearBottomX = 0.0;
	if(pLaneParameterOneSide->Available() == true) {
		dNearBottomX = pLaneParameterOneSide->LaneBoundaryPositionOnRoad(iK, dNearBottomZ);
	} else {
		return NULL;
	}
	double dNearBottomXCenter = dNearBottomX;
	double dNearBottomXMin = dNearBottomXCenter - g_dbOffsetSearchMarginTrack;
	double dNearBottomXMax = dNearBottomXCenter + g_dbOffsetSearchMarginTrack;

	double dNearTopZ = pNearRegion->get_top();
	double dNearTopX = 0.0;
	if(pLaneParameterOneSide->Available() == true) {
		dNearTopX = pLaneParameterOneSide->LaneBoundaryPositionOnRoad(iK, dNearTopZ);
	} else {
		return NULL;
	}
	double dNearTopXCenter = dNearTopX;
	double dNearTopXMin = dNearTopXCenter - g_dbOffsetSearchMarginTrack;
	double dNearTopXMax = dNearTopXCenter + g_dbOffsetSearchMarginTrack;

	LaneMarkerLineSequences *pLMLSs = getLaneMarkerLineSequences(iLR);
	int iBottomOffsetCenter = pNearSide->transform_Xvehicle_to_Jroad(dNearBottomXCenter);	// //(lateral)
	int iBottomOffsetMin = pNearSide->transform_Xvehicle_to_Jroad(dNearBottomXMin);	// //(lateral)
	int iBottomOffsetMax = pNearSide->transform_Xvehicle_to_Jroad(dNearBottomXMax);	// //(lateral)
	int iTopOffsetCenter = pNearSide->transform_Xvehicle_to_Jroad(dNearTopXCenter);	// //(lateral)
	int iTopOffsetMin = pNearSide->transform_Xvehicle_to_Jroad(dNearTopXMin);	// //(lateral)
	int iTopOffsetMax = pNearSide->transform_Xvehicle_to_Jroad(dNearTopXMax);	// //(lateral)
	LaneMarkerLineSequence *pLMLS = searchLaneMarkerLineSequenceForLaneBoundary(pLMLSs, iUD, iBottomOffsetCenter, iBottomOffsetMin, iBottomOffsetMax, iTopOffsetCenter, iTopOffsetMin, iTopOffsetMax);

	if(pLMLS != NULL) {
		LaneMarker *pNewNearLM = new LaneMarker(pLMLS->Near(), pLMLS->Near());	// //
		pNearSide->setLaneBoundary(pNewNearLM);	////
		pNearSide->setFoundNowFlag();		// //
		pNearSide->LaneBoundaryType(0, CHX_LBT_LANEMARKERLINE);
		LaneMarker *pNewFarLM = new LaneMarker(pLMLS->Far(), pLMLS->Far());	// //
		pFarSide->setLaneBoundary(pNewFarLM);	////
		pFarSide->setFoundNowFlag();		// //
		pFarSide->LaneBoundaryType(0, CHX_LBT_LANEMARKERLINE);
	}
	return pLMLS;
}
LaneMarkerLine *LaneArea::searchLaneMarkerLineForLaneBoundaryByParameter(int iNF, int iLR, int iUD)					// 09
{
	int iK = iLR == CHX_LR_LEFT ? -1 : 1;			// // -1, // 1

	LaneParameterOneSide *pLaneParameterOneSide = getLaneParameterOneSide(iLR);
	if(pLaneParameterOneSide == NULL)	return NULL;
	LaneRegion *pRegion = get_region(iNF);
	if(pRegion == NULL)	return NULL;
	LaneSide *pSide = pRegion->get_side(iLR);
	double dBottomZ = pRegion->get_bottom();
	double dBottomX = 0.0;
	if(pLaneParameterOneSide->Available() == true) {
		dBottomX = pLaneParameterOneSide->LaneBoundaryPositionOnRoad(iK, dBottomZ);
	} else {
		return NULL;
	}
	double dBottomXCenter = dBottomX;
	double dBottomXMin = dBottomXCenter - g_dbOffsetSearchMarginTrack;
	double dBottomXMax = dBottomXCenter + g_dbOffsetSearchMarginTrack;

	double dTopZ = pRegion->get_top();
	double dTopX = 0.0;
	if(pLaneParameterOneSide->Available() == true) {
		dTopX = pLaneParameterOneSide->LaneBoundaryPositionOnRoad(iK, dTopZ);
	} else {
		return NULL;
	}
	double dTopXCenter = dTopX;
	double dTopXMin = dTopXCenter - g_dbOffsetSearchMarginTrack;
	double dTopXMax = dTopXCenter + g_dbOffsetSearchMarginTrack;
	LaneMarkerLines *pLMLs = pSide->get_lane_marker_lines(iUD);
	int iBottomOffsetCenter = pSide->transform_Xvehicle_to_Jroad(dBottomXCenter);	// //(lateral)
	int iBottomOffsetMin = pSide->transform_Xvehicle_to_Jroad(dBottomXMin);	// //(lateral)
	int iBottomOffsetMax = pSide->transform_Xvehicle_to_Jroad(dBottomXMax);	// //(lateral)
	int iTopOffsetCenter = pSide->transform_Xvehicle_to_Jroad(dTopXCenter);	// //(lateral)
	int iTopOffsetMin = pSide->transform_Xvehicle_to_Jroad(dTopXMin);	// //(lateral)
	int iTopOffsetMax = pSide->transform_Xvehicle_to_Jroad(dTopXMax);	// //(lateral)
	LaneMarkerLine *pLML = searchLaneMarkerLineForLaneBoundary(pLMLs, iBottomOffsetCenter, iBottomOffsetMin, iBottomOffsetMax, iTopOffsetCenter, iTopOffsetMin, iTopOffsetMax);

	if(pLML != NULL) {	// ////	// (b)-(a)-LM-
		LaneMarker *pNewLM = new LaneMarker(pLML, pLML);	// 
		pSide->setLaneBoundary(pNewLM);	////
		pSide->setFoundNowFlag();		// //
		pSide->LaneBoundaryType(0, CHX_LBT_LANEMARKERLINE);
	}
	return pLML;
}
LaneMarkerLine *LaneArea::searchLaneMarkerLineForLaneBoundaryByNearerRegionAndParameter(int iNF, int iLR, int iUD)	// 10
{
	LaneRegion *pRegion = get_region(iNF);
	if(pRegion == NULL)	return NULL;
	LaneRegion *pNearerRegion = get_region(iNF - 1);
	if(pNearerRegion == NULL)	return NULL;

	LaneMarker *pNearerLM = pNearerRegion->get_side(iLR)->getLaneBoundary();
	if(pNearerLM == NULL)	return NULL;

	int iK = iLR == CHX_LR_LEFT ? -1 : 1;			// // -1, // 1

	LaneSide *pSide = pRegion->get_side(iLR);		// //	
	if(pSide == NULL)	return NULL;
	LaneMarkerLines *pLMLs = pSide->get_lane_marker_lines(iUD);

	int iOffsetCenter = pNearerLM->get_lane_marker_line(iUD)->get_offset() + pNearerLM->get_lane_marker_line(iUD)->get_yaw();
	int iOffsetMin = iOffsetCenter - (int)(g_dbOffsetSearchMarginSequence / get_region(iNF)->get_side(iNF)->get_width() * get_region(iNF)->get_side(iNF)->get_road_image_width());
	int iOffsetMax = iOffsetCenter + (int)(g_dbOffsetSearchMarginSequence / get_region(iNF)->get_side(iNF)->get_width() * get_region(iNF)->get_side(iNF)->get_road_image_width());
	int iYawCenter = pNearerLM->get_lane_marker_line(iUD)->get_yaw();
	double dYawDiff = calcYawDiffNearAndFar(iLR, CHX_NF_NEAR, CHX_NF_FAR);
	int iYawDiff = (int)((tan(dYawDiff) *pSide->get_height() / pSide->get_width()) * pSide->get_road_image_width());
	iYawCenter += iYawDiff;
	int iYawMargin = (int)((tan(g_dbYawSearchMarginSequence) * pSide->get_height() / pSide->get_width()) * pSide->get_road_image_width());
	int iYawMin = iYawCenter - iYawMargin;				// //
	int iYawMax = iYawCenter + iYawMargin;				// //
	LaneMarkerLine *pLML = searchLaneMarkerLineForLaneBoundary(pLMLs, iOffsetCenter, iOffsetMin, iOffsetMax, iYawCenter, iYawMin, iYawMax);

	if(pLML != NULL) {
		LaneMarker *pNewLM = new LaneMarker(pLML, pLML);	// //
		pSide->setLaneBoundary(pNewLM);	////
		pSide->setFoundNowFlag();		// //
		pSide->LaneBoundaryType(0, CHX_LBT_LANEMARKERLINE);
	}

	return pLML;
}

double searchMedian(FlexArray<double> *pfadDeviations)
{
	if(pfadDeviations->get_number() == 0)	return 0;
	FlexArray<double> pfadTmp;
	for(int iIdx = 0; iIdx < pfadDeviations->get_number(); iIdx++) {
		pfadTmp.add(pfadDeviations->get(iIdx));
	}
	for(int iIdx0 = 0; iIdx0 < pfadTmp.get_number() - 1; iIdx0++) {
		for(int iIdx1 = iIdx0 + 1; iIdx1 < pfadTmp.get_number(); iIdx1++) {
			if(pfadTmp.get(iIdx0) > pfadTmp.get(iIdx1)) {
				double dTmp = pfadTmp.get(iIdx0);
				pfadTmp.set(iIdx0, pfadTmp.get(iIdx1));
				pfadTmp.set(iIdx1, dTmp);
			}
		}
	}
	double dMedian = pfadTmp.get(pfadTmp.get_number()/ 2);

	return dMedian;
}

bool LaneArea::searchLaneBoundaryPointsInNearArea(CamParam *pCamParam, LaneMarkerPoints *pUpEdgePoints, LaneMarkerPoints *pDownEdgePoints)
{
	if(pCamParam == NULL)	return false;
	if(pUpEdgePoints == NULL)	return false;
	if(pDownEdgePoints == NULL)	return false;

	double dZMax = CS4_FAR_TOPY ;	//		// ////
	double dZMid = CS4_NEAR_TOPY;	//		// ////
	double dZMin = CS4_NEAR_BASEY;	//		// ////
	int iIsrcMin =  calcHorizontalLineOfDepth(pCamParam, dZMax);	// //
	int iIsrcMid =  calcHorizontalLineOfDepth(pCamParam, dZMid);	// //
	int iIsrcMax =  calcHorizontalLineOfDepth(pCamParam, dZMin);	// //

	for(int iLR = 0; iLR < CHX_LR_NUM; iLR++) {		// //
		int iUD = iLR == CHX_LR_LEFT ? CHX_UD_DOWN : CHX_UD_UP;
		LaneParameterOneSide *pLaneParameterOneSide = getLaneParameterOneSide(iLR);		// //
		LaneMarkerPoints *pNewLBPs = new LaneMarkerPoints;		// 
		setLaneBoundaryPointsInNearArea(iLR, pNewLBPs);			// ////
		if(get_region(CHX_NF_NEAR)->get_side(iLR)->getComplexLaneBoundary()->Mode() > CLBT_SINGLE) continue;

		FlexArray<double>	fadDeviation;						// //
		int iPreviousFoundIsrc = -1;	// //
		int iPreviousFoundJsrc = -1;	// //
		LaneMarkerPoints *pEdgePoints = (iLR == CHX_LR_LEFT) ? pDownEdgePoints : pUpEdgePoints;	// ?
		int iK = (iLR == CHX_LR_LEFT) ? -1 : 1;	// //
		int iIdx = 0;
		int iPointNum= pEdgePoints->get_lane_marker_point_number();	// 
		while(iIdx < iPointNum) {	// //
			LaneMarkerPoint *pLMP = pEdgePoints->get_lane_marker_point(iIdx);
			int iIsrc = pLMP->get_Isrc();
			int iJsrc = pLMP->get_Jsrc();
			if(iIsrc < iIsrcMin)	break;	// //
			if(iPreviousFoundIsrc < 0) {
				if(iIsrc < iIsrcMid)	break;	////
			}
			int iMaxDiffFromPreviousFoundIsrc = 10;	// //
			int iMaxDiffFromPreviousFoundJsrc = 10;	// //
			if(	iPreviousFoundIsrc >= 0
				&&	iIsrc	<	iIsrcMid	// //201111003
				&&	(	abs(iPreviousFoundIsrc - iIsrc) > iMaxDiffFromPreviousFoundIsrc	// //
				)
				) {
					break;	// //
			}
			int iJsrcC = 0;
			if(pLaneParameterOneSide->Available() == true) {
				iJsrcC = (int)pLaneParameterOneSide->LaneBoundaryPositionOnImagePixel(pCamParam, iIsrc);	// //
			} else {
				break;	// //
			}
			int iWidthOfPixel = (int)getWidthOfPixel(pCamParam, iIsrc);
			double dSearchMargin = SearchMarginInNearArea();		// //
			int iJsrcMin = (int)(iJsrcC - dSearchMargin / iWidthOfPixel);	
			int iJsrcMax = (int)(iJsrcC + dSearchMargin / iWidthOfPixel);
			if(	0
				||	(	(iJsrcMin <= iJsrc && iJsrc <= iJsrcMax)
				)
				) {		// //
					pNewLBPs->add_lane_marker_point(pLMP);
					double dDeviation = abs(iJsrc - iJsrcC) * iWidthOfPixel;
					fadDeviation.add(dDeviation);
					iPreviousFoundIsrc = iIsrc;
					iPreviousFoundJsrc = iJsrc;
			}
			iIdx++;	// //
		}
		{
			pNewLBPs->calculate_average_edge_strength();
			double dAverageEdgeStrength = pNewLBPs->get_average_of_edge_strength();

			int iNumber = pNewLBPs->get_lane_marker_point_number();
			for(int iIdx = iNumber - 1; iIdx >= 0; iIdx--) {
				LaneMarkerPoint *pLMP = pNewLBPs->get_lane_marker_point(iIdx);
				if(pLMP == NULL)	continue;
				if(fabs(pLMP->get_edge_strength()) < fabs(dAverageEdgeStrength * EDGESTRENGTH_RATIO_TO_ELIMINATE)) {
					pNewLBPs->delete_lane_marker_point(iIdx);
					fadDeviation.remove(iIdx);
				}
			}
		}


		int iSelectedPointNumber = pNewLBPs->get_lane_marker_point_number();	// 
		if(	iSelectedPointNumber < MININUM_POINT_NUMBER_FOR_NEAR_AREA) {
			getLaneBoundaryPointsInNearArea(iLR)->clear_reset();
			continue;
		}
		int iDeletedPointNumber = 0;
		double dMedian = searchMedian(&fadDeviation);
		for(int iIdx = pNewLBPs->get_lane_marker_point_number() - 1; iIdx >= 0; iIdx--) {
			double dMaxDiffFromMedianDeviation = MaxDiffFromMedianDeviationInNearArea();
			if((fadDeviation.get(iIdx) - dMedian) > dMaxDiffFromMedianDeviation) {	// //
				pNewLBPs->delete_lane_marker_point(iIdx);
				iDeletedPointNumber++;
			}
		}
		double dMaximumRatioOfRejectedPoints = MAXIMUM_RATIO_OF_REJECTED_POINTS;	// //
		if(	(iSelectedPointNumber > 0)
			&&	((1.0 * iDeletedPointNumber / iSelectedPointNumber) > dMaximumRatioOfRejectedPoints)	////
			) {
				getLaneBoundaryPointsInNearArea(iLR)->clear_reset();
				continue;
		}
	}
	return true;
}

LaneMarkerPoints *LaneArea::mergeLaneMarkerPointsHead(LaneMarkerPoints *pBaseLMPs, LaneMarkerPoints *pLMPs)
{
	LaneMarkerPoints *pNewLMPs = new LaneMarkerPoints();	// ////
	int iIsrcMin = -1;
	if(pBaseLMPs != NULL && pBaseLMPs->get_lane_marker_point_number() > 0) {
		iIsrcMin = pBaseLMPs->get_lane_marker_point(0)->get_Isrc();	////
	}
	if(pLMPs != NULL) {
		for(int iIdx = 0; iIdx < pLMPs->get_lane_marker_point_number(); iIdx++) {
			LaneMarkerPoint *pLMP = pLMPs->get_lane_marker_point(iIdx);
			if(pLMP == NULL)	continue;
			if(iIsrcMin >= 0 && pLMP->get_Isrc() <= iIsrcMin)	break;	// //
			pNewLMPs->add_lane_marker_point(pLMP);	// //
		}
	}
	if(pBaseLMPs != NULL) {
		for(int iIdx = 0; iIdx < pBaseLMPs->get_lane_marker_point_number(); iIdx++) {
			LaneMarkerPoint *pLMP = pBaseLMPs->get_lane_marker_point(iIdx);
			if(pLMP == NULL)	continue;
			pNewLMPs->add_lane_marker_point(pLMP);	// //
		}
	}
	return pNewLMPs;	// //
}
// //
LaneMarkerPoints *LaneArea::mergeLaneMarkerPointsTail(LaneMarkerPoints *pBaseLMPs, LaneMarkerPoints *pLMPs)
{
	LaneMarkerPoints *pNewLMPs = new LaneMarkerPoints();	// //
	if(pBaseLMPs != NULL) {
		for(int iIdx = 0; iIdx < pBaseLMPs->get_lane_marker_point_number(); iIdx++) {
			LaneMarkerPoint *pLMP = pBaseLMPs->get_lane_marker_point(iIdx);
			if(pLMP == NULL)	continue;
			pNewLMPs->add_lane_marker_point(pLMP);// //
		}
	}
	int iIsrcMax = 10000;	// //
	if(pBaseLMPs != NULL && pBaseLMPs->get_lane_marker_point_number() > 0) {
		iIsrcMax = pBaseLMPs->get_lane_marker_point(pBaseLMPs->get_lane_marker_point_number() - 1)->get_Isrc();	// //
	}
	if(pLMPs != NULL) {
		for(int iIdx = 0; iIdx < pLMPs->get_lane_marker_point_number(); iIdx++) {
			LaneMarkerPoint *pLMP = pLMPs->get_lane_marker_point(iIdx);
			if(pLMP == NULL)	continue;
			if(pLMP->get_Isrc() >= iIsrcMax)	continue;	// //
			pNewLMPs->add_lane_marker_point(pLMP);	// //
		}
	}
	return pNewLMPs;	// //
}
bool LaneArea::select_lane_markers(LaneParameterOneSide *pLeftLaneParameter, LaneParameterOneSide *pRightLaneParameter){
	if(pLeftLaneParameter == NULL)	return false;
	if(pRightLaneParameter == NULL)	return false;

	LaneRegion *pNearRegion = this->get_region(CHX_NF_NEAR);	// //
	if(pNearRegion == NULL)	return false;
	LaneRegion *pFarRegion = get_region(CHX_NF_FAR);	// //
	if(pFarRegion == NULL)	return false;

	// ////?//////
	double dMinimumLaneMarkerWidth = CS4_MINIMUM_LINEWIDTH;////////[mm]////
	double dMaximumLaneMarkerWidth = CS4_MAXIMUM_LINEWIDTH;		//// [mm]/////
	double dMinimumAverageEdgeStrengthRatio = MINIMUM_AVERAGE_EDGE_STRENGTH_RATIO;	// //

	//////////////
	int iWidthOfPixel= (int)(pNearRegion->get_side(CHX_LR_LEFT)->get_width() / pNearRegion->get_side(CHX_LR_LEFT)->get_road_image_width());

	LaneMarkerPair *pLMPairSelect = NULL;
	for(int iLR = 0; iLR < CHX_LR_NUM; iLR++) {	// //
		LaneSide *pNearSide = pNearRegion->get_side(iLR);	// ////
		if(pNearSide == NULL)	continue;
		LaneSide *pFarSide = pFarRegion->get_side(iLR);		// 	//
		if(pFarSide == NULL) continue;
		int iUD = iLR == CHX_LR_LEFT ? CHX_UD_DOWN : CHX_UD_UP;	// //, // //
		int iK = iLR == CHX_LR_LEFT ? -1 : 1;			// // -1, // 1
		switch(this->get_lane_boundary_detect_type(iLR)) {	//// ////
		case	CHX_LBT_NONE:	// //	(a)///////////
			{
				LaneMarker *pNearLM = NULL;
// 				if(UseSelectLaneBoundaryByComplexMode) {
// 					pNearLM = pNearSide->selectLaneMarkerAtComplexLaneBoundary(iUD);
// 					if(pNearLM != NULL) {
// 						LaneMarker *pNewLM = new LaneMarker(pNearLM);	// //
// 						pNearSide->setLaneBoundary(pNewLM);	// //////
// 						pNearSide->setFoundNowFlag();		// //
// 						pNearSide->LaneBoundaryType(0, CHX_LBT_LANEMARKER);
// 					}
// 				}
 				if(pNearSide->getComplexLaneBoundary()->Mode() <= CLBT_SINGLE) {	// beginning of non-complex mode
 					if(pNearLM != NULL) {	////
 						LaneMarker *pNewLM = new LaneMarker(pNearLM);	// //
 						pNearSide->setLaneBoundary(pNewLM);	// //////
 						pNearSide->setFoundNowFlag();		// //
 						pNearSide->LaneBoundaryType(0, CHX_LBT_LANEMARKER);
 					} else if(pLMPairSelect != NULL) {	//////////////////
 						pNearLM = pLMPairSelect->get_lane_marker(iLR);
 					}
 					if(pNearLM == NULL) {	////
 						pNearLM = searchLaneMarkerForLaneBoundaryByInitialParameter(CHX_NF_NEAR,iLR);	// (1) LM-
 					}
 				} // end of non-complex mode
				if(pNearLM != NULL) {	// //	(a)-(a)-LM-
					LaneMarker *pFarLM = searchLaneMarkerForLaneBoundaryByNearerRegion(CHX_NF_FAR, iLR);	// (3) LM-LM
					if(pFarLM == NULL) {	// LM-NONE
						pFarSide->setLaneBoundary(NULL);		// //
						pFarSide->clearFoundNowFlag();			// //
						pFarSide->LaneBoundaryType(0, CHX_LBT_NONE);// ////
					}
				} else {	// //
					LaneMarkerLineSequence *pLMLS = searchLaneMarkerLineSequenceForLaneBoundaryByInitialParameter(iLR);	// (2) EDGE-EDGE
					if(pLMLS == NULL) {	// NONE-NONE
						pNearSide->setLaneBoundary(NULL);		// ////
						pNearSide->clearFoundNowFlag();			// //
						pNearSide->LaneBoundaryType(0, CHX_LBT_NONE);// ////
						pFarSide->setLaneBoundary(NULL);		// ////
						pFarSide->clearFoundNowFlag();			// //
						pFarSide->LaneBoundaryType(0, CHX_LBT_NONE);// ////
					}
				}
			}
			break;
		case	CHX_LBT_LANEMARKER:	// //	// (b)////////
			{
				LaneMarker *pNearLM = NULL;
// 				if(UseSelectLaneBoundaryByComplexMode) {
// 					pNearLM = pNearSide->selectLaneMarkerAtComplexLaneBoundary(iUD);
// 					if(pNearLM != NULL) {
// 						LaneMarker *pNewLM = new LaneMarker(pNearLM);	// 
// 						pNearSide->setLaneBoundary(pNewLM);	// ////
// 						pNearSide->setFoundNowFlag();		// //
// 						pNearSide->LaneBoundaryType(0, CHX_LBT_LANEMARKER);
// 					}
// 				}
 				if(pNearSide->getComplexLaneBoundary()->Mode() <= CLBT_SINGLE) {	// beginning of non-complex mode
 					if(pNearLM != NULL) {	// //
 						LaneMarker *pNewLM = new LaneMarker(pNearLM);	// //
 						pNearSide->setLaneBoundary(pNewLM);	// //////
 						pNearSide->setFoundNowFlag();		// //
 						pNearSide->LaneBoundaryType(0, CHX_LBT_LANEMARKER);
 					} else {	// //
 						pNearLM = searchLaneMarkerForLaneBoundaryByPreviousFrame(CHX_NF_NEAR, iLR);	// (4) LM-
 					}
 					if(pNearLM == NULL) {	// //
 						pNearLM = searchLaneMarkerForLaneBoundaryByPreviousFurtherFrame(CHX_NF_NEAR, iLR);	// (4) LM-
 					}
 				} // end of non-complex mode

				if(pNearLM != NULL) {	// ////
					LaneMarker *pFarLM = searchLaneMarkerForLaneBoundaryByNearerRegion(CHX_NF_FAR, iLR);	// (3) LM-LM
					if(pFarLM == NULL) {	// LM-NONE
						pFarSide->setLaneBoundary(NULL);		// ////
						pFarSide->clearFoundNowFlag();			///
						pFarSide->LaneBoundaryType(0, CHX_LBT_NONE);// ////
					}
				} else {	// //	// NONE-
					pNearSide->setLaneBoundary(NULL);		// ////
					pNearSide->clearFoundNowFlag();			// //
					pNearSide->LaneBoundaryType(0, CHX_LBT_NONE);// ////
					LaneMarker *pFarLM = searchLaneMarkerForLaneBoundaryByPreviousFrame(CHX_NF_FAR, iLR);	// (4) NONE-LM
					if(pFarLM == NULL) {	// NONE-NONE
						pFarSide->setLaneBoundary(NULL);		// ////
						pFarSide->clearFoundNowFlag();			// //
						pFarSide->LaneBoundaryType(0, CHX_LBT_NONE);// ////
					}
				}
			}
			break;
		case	CHX_LBT_LANEMARKERLINE:		// //	// (c)//////////////
			{
				// //
				LaneMarker *pNearLM = NULL;
// 				if(UseSelectLaneBoundaryByComplexMode) {
// 					pNearLM = pNearSide->selectLaneMarkerAtComplexLaneBoundary(iUD);
// 					if(pNearLM != NULL) {
// 						LaneMarker *pNewLM = new LaneMarker(pNearLM);	// 
// 						pNearSide->setLaneBoundary(pNewLM);	// ////
// 						pNearSide->setFoundNowFlag();		// //
// 						pNearSide->LaneBoundaryType(0, CHX_LBT_LANEMARKER);
// 					}
// 				}
 				if(pNearSide->getComplexLaneBoundary()->Mode() <= CLBT_SINGLE) {	// beginning of non-complex mode
 					if(pNearLM != NULL) {	// //
 						LaneMarker *pNewLM = new LaneMarker(pNearLM);	// 
 						pNearSide->setLaneBoundary(pNewLM);	// ////
 						pNearSide->setFoundNowFlag();		// //
 						pNearSide->LaneBoundaryType(0, CHX_LBT_LANEMARKER);
 					} else {	// //
 						pNearLM = searchLaneMarkerForLaneBoundaryByInitialParameter(CHX_NF_NEAR,iLR);	// (1) LM-
 					}
 				}	// end of non-complex mode

				if(pNearLM != NULL) {	// //	LM-
					LaneMarker *pFarLM = searchLaneMarkerForLaneBoundaryByNearerRegion(CHX_NF_FAR, iLR);	// (3) LM-LM
					if(pFarLM == NULL) {	// LM-NONE
						pFarSide->setLaneBoundary(NULL);		// ////
						pFarSide->clearFoundNowFlag();			////
						pFarSide->LaneBoundaryType(0, CHX_LBT_NONE);// ////
					}
				} else {	// //
					LaneMarkerLineSequence *pLMLS = searchLaneMarkerLineSequenceForLaneBoundaryByPreviousFrame(iLR);	// (5) EDGE-EDGE
					if(pLMLS == NULL) {	// NONE-NONE
						pNearSide->setLaneBoundary(NULL);		// ////
						pNearSide->clearFoundNowFlag();			// //
						pNearSide->LaneBoundaryType(0, CHX_LBT_NONE);// ////
						pFarSide->setLaneBoundary(NULL);		// ////
						pFarSide->clearFoundNowFlag();			// //
						pFarSide->LaneBoundaryType(0, CHX_LBT_NONE);// ////
					}
				}
			}
			break;
		case	CHX_LBT_LANEMARKER_TRACK:	// //	(d)//////////////
			{
				LaneMarker *pNearLM = NULL;
				if(pNearLM == NULL) {	// //
					if(SearchInDefaultLanePositionFlag(iLR) == true) {
						pNearLM = searchLaneMarkerForLaneBoundaryByInitialParameter(CHX_NF_NEAR,iLR);	// (1) LM-
						if(pNearLM)	break;
					}
				}
// 				if(UseSelectLaneBoundaryByComplexMode) {
// 					if(pNearLM == NULL) {
// 						pNearLM = pNearSide->selectLaneMarkerAtComplexLaneBoundary(iUD);
// 						if(pNearLM != NULL) {
// 							LaneMarker *pNewLM = new LaneMarker(pNearLM);	// 
// 							pNearSide->setLaneBoundary(pNewLM);	// ////
// 							pNearSide->setFoundNowFlag();		// //
// 							pNearSide->LaneBoundaryType(0, CHX_LBT_LANEMARKER);
// 						}
// 					}
// 				}
 				if(pNearSide->getComplexLaneBoundary()->Mode() <= CLBT_SINGLE) {	// beginning of non-complex mode
 					if(pNearLM != NULL) {	// //
 						LaneMarker *pNewLM = new LaneMarker(pNearLM);	// 
 						pNearSide->setLaneBoundary(pNewLM);	// ////
 						pNearSide->setFoundNowFlag();		// //
 						pNearSide->LaneBoundaryType(0, CHX_LBT_LANEMARKER_TRACK);
 					}
 
 					if(pNearLM == NULL) {	////////
 						pNearLM = searchLaneMarkerForLaneBoundaryByParameter(CHX_NF_NEAR, iLR);	// (6) LM-
 					}
 					if(pNearLM == NULL)	{	// //////
 						pNearLM = searchLaneMarkerForLaneBoundaryByPreviousFrame(CHX_NF_NEAR, iLR);
 					}	// (4) LM-
 				}	// end of non-complex mode

				if(pNearLM != NULL) {	// //////	// LM-
					LaneMarker *pFarLM = searchLaneMarkerForLaneBoundaryByNearerRegionAndParameter(CHX_NF_FAR, iLR);	// (7) LM-LM
					if(pFarLM == NULL)	{
						pFarLM = searchLaneMarkerForLaneBoundaryByNearerRegion(CHX_NF_FAR, iLR);
					}	// (3) LM-LM
					if(pFarLM == NULL) {	// // // LM-NONE
						pFarSide->setLaneBoundary(NULL);
						pFarSide->clearFoundNowFlag();	// //
						pFarSide->LaneBoundaryType(0, CHX_LBT_NONE);
					}
				} else {	// //		// NONE-
					pNearSide->setLaneBoundary(NULL);	// //////
					pNearSide->clearFoundNowFlag();		////
					pNearSide->LaneBoundaryType(0, CHX_LBT_NONE);

					LaneMarker *pFarLM = searchLaneMarkerForLaneBoundaryByParameter(CHX_NF_FAR, iLR);	// (6) NONE-LM
					if(pFarLM == NULL)	{	pFarLM = searchLaneMarkerForLaneBoundaryByPreviousFrame(CHX_NF_FAR, iLR);
					}	// (4) NONE-LM
					if(pFarLM == NULL) {	// //	// NONE-NONE
						pFarSide->setLaneBoundary(NULL);
						pFarSide->clearFoundNowFlag();	// //
						pFarSide->LaneBoundaryType(0, CHX_LBT_NONE);
					}
				}
			}
			break;
		case	CHX_LBT_LANEMARKERLINE_TRACK:	///////////// (e)///////////////////
			{
				LaneMarker *pNearLM = NULL;
				if(pNearLM == NULL) {	// ////
					if(SearchInDefaultLanePositionFlag(iLR) == true) {
						pNearLM = searchLaneMarkerForLaneBoundaryByInitialParameter(CHX_NF_NEAR,iLR);	// (1) LM-
						if(pNearLM)	break;
					}
				}
				if(pNearLM == NULL) {
// 					if(UseSelectLaneBoundaryByComplexMode) {
// 						if(pNearSide->getComplexLaneBoundary()->Mode() > CLBT_SINGLE) {
// 							pNearLM = pNearSide->selectLaneMarkerAtComplexLaneBoundary(iUD);
// 							if(pNearLM != NULL) {
// 								LaneMarker *pNewLM = new LaneMarker(pNearLM);	// 
// 								pNearSide->setLaneBoundary(pNewLM);	// ////
// 								pNearSide->setFoundNowFlag();		// //
// 								pNearSide->LaneBoundaryType(0, CHX_LBT_LANEMARKER);
// 								pFarSide->setLaneBoundary(NULL);		// ////
// 								pFarSide->clearFoundNowFlag();			// //
// 								pFarSide->LaneBoundaryType(0, CHX_LBT_NONE);// ////
// 							} else {
// 								pNearSide->setLaneBoundary(NULL);		// ////
// 								pNearSide->clearFoundNowFlag();			// //
// 								pNearSide->LaneBoundaryType(0, CHX_LBT_NONE);// ////
// 								pFarSide->setLaneBoundary(NULL);		// ////
// 								pFarSide->clearFoundNowFlag();			// //
// 								pFarSide->LaneBoundaryType(0, CHX_LBT_NONE);// ////
// 							}
// 							break;
// 						}
// 					}
				}

				int iEdgeDir = pNearSide->LaneBoundaryType(1);
				iUD = iEdgeDir == CHX_LBT_UP ? CHX_UD_UP : CHX_UD_DOWN;	
				LaneMarkerLineSequence *pLMLS = searchLaneMarkerLineSequenceForLaneBoundaryByParameter(iLR);	// (8) EDGE-EDGE
				if(pLMLS == NULL)	{
					pLMLS = searchLaneMarkerLineSequenceForLaneBoundaryByPreviousFrame(iLR);
				}	// (5) EDGE-EDGE

				if(pLMLS == NULL) {	// //
					LaneMarkerLine *pNearLML = searchLaneMarkerLineForLaneBoundaryByParameter(CHX_NF_NEAR, iLR, iUD);	// (9) EDGE-
					if(pNearLML == NULL) {	// //
						pNearSide->setLaneBoundary(NULL);	// ////
						pNearSide->clearFoundNowFlag();		// //
						pNearSide->LaneBoundaryType(0, CHX_LBT_NONE);

						LaneMarkerLine *pFarLML = searchLaneMarkerLineForLaneBoundaryByParameter(CHX_NF_FAR, iLR, iUD);	// (9) NONE-EDGE
						if(pFarLML == NULL) {	// NONE-NONE
							pFarSide->setLaneBoundary(NULL);	// ////
							pFarSide->clearFoundNowFlag();		// //
							pFarSide->LaneBoundaryType(0, CHX_LBT_NONE);
						}
					} else {	// //
						LaneMarkerLine *pFarLML = searchLaneMarkerLineForLaneBoundaryByNearerRegionAndParameter(CHX_NF_FAR, iLR, iUD);	// (10) EDGE-EDGE
						if(pFarLML == NULL) {			// EDGE-NONE
							pFarSide->setLaneBoundary(NULL);	// ////
							pFarSide->clearFoundNowFlag();		// //
							pFarSide->LaneBoundaryType(0, CHX_LBT_NONE);
						}
					}
				}
			}
			break;
		default:	/////////
			break;
		}
	}

	return true;
}

bool LaneArea::pickup_lane_boundary_points(CamParam *pCamParam, LaneMarkerPoints *pUpEdgePoints, LaneMarkerPoints *pDownEdgePoints){
	//TODO:add your code here//
	if (pCamParam == NULL)
		return false;
	if (pUpEdgePoints == NULL)
		return false;
	if (pDownEdgePoints == NULL)
		return false;

	for (int iNF = 0; iNF < CHX_NF_NUM; iNF++) { //////
		LaneRegion *pRegion = this->get_region(iNF);
		if (pRegion == NULL)
			continue;
		int iProcLineIndexMin =(iNF == CHX_NF_NEAR) ?(this->get_line_number_far_area() + this->get_region(CHX_NF_FAR)->get_line_number()) :	this->get_line_number_far_area();
		int iProcLineIndexMax = iProcLineIndexMin + this->get_region(iNF)->get_line_number() - 1;
		int iIsrcMin = iProcLineIndexMin;
		int iIsrcMax = iProcLineIndexMax;
		pRegion->pickup_lane_boundary_points(pCamParam, pUpEdgePoints,
				pDownEdgePoints, iIsrcMin, iIsrcMax);	//////////
	}

	return true;
}

bool LaneArea::select_lane_boundary_points_to_estimate_lane_parameter(CamParam *pCamParam)
{
  //TODO:add your code here//
	for (int iLR = 0; iLR < CHX_LR_NUM; iLR++) {		////
		LaneMarkerPoints *pNewLBPs = new LaneMarkerPoints;		////

		this->set_lane_boundary_points_to_estimate_lane_parameter(iLR,pNewLBPs);

		LaneMarkerPoints *pLBPs = this->get_lane_boundary_points(iLR);
		if (pLBPs == NULL)
			continue;
		int iPointNumber = pLBPs->get_lane_marker_point_number();
		int iSelectedNumber = LANE_BOUNDARY_POINTS_NUMBER_FOR_PARAMETER_ESTIMATION;
		if (iSelectedNumber > iPointNumber) {
			iSelectedNumber = iPointNumber;
		}
		if (iSelectedNumber <= 1)
			continue;	////
		for (int iIdx = 0; iIdx < iSelectedNumber; iIdx++) {
			int iSelectedIdx = ((iPointNumber - 1) * iIdx)
					/ (iSelectedNumber - 1);	////
			LaneMarkerPoint *pLMP = pLBPs->get_lane_marker_point(iSelectedIdx);
			pNewLBPs->add_lane_marker_point(pLMP);
		}
	}
	return true;
}

bool LaneArea::decide_lane_boundary_line_type(void) {

	//TODO:add your code here//
	double dMinRatioOfSolidLine = MIN_RATIO_OF_SOLID_LINE_FOR_ONELINE_FILTER;
	int iLineNumberOfNearRegion =
			this->get_region(CHX_NF_NEAR)->get_line_number();
	for (int iLR = 0; iLR < CHX_LR_NUM; iLR++) {	////////
		int iUD = iLR == CHX_LR_LEFT ? CHX_UD_DOWN : CHX_UD_UP;
		LaneRegion *pLR = this->get_region(CHX_NF_NEAR);
		if(pLR == NULL)
			continue;
		LaneSide *pSideNear = pLR->get_side(iLR);
		if (pSideNear == NULL)
			continue;
		LaneMarker* pLM = pSideNear->getLaneBoundary();
		if (pLM == NULL)
			continue;
		LaneMarkerLine* pLML = pLM->get_lane_marker_line(iUD);
		if (pLML == NULL)
			continue;
		LaneMarkerPoints *pLMPs = pLML->get_lane_marker_points();
		if (pLMPs == NULL)
			continue;
		int iNearPointNum = pLMPs->get_lane_marker_point_number();

//		int iFarPointNum =
//				pSideFar->get_lane_boundary()->get_lane_marker_line(iUD)->get_lane_marker_points()->get_lane_marker_point_number();
		double dRatioOfDetecedPointNumber = (1.0 * iNearPointNum)
				/ (iLineNumberOfNearRegion);
		double dRatioOfDetecedPointNumberOfNearRegion = (1.0 * iNearPointNum)
				/ iLineNumberOfNearRegion;
		if (dRatioOfDetecedPointNumber >= dMinRatioOfSolidLine) {
			this->set_lane_boundary_line_type(iLR, CHX_LBT_SOLID);
		} else {
			this->set_lane_boundary_line_type(iLR, CHX_LBT_DASH);
		}
	}
	return true;
}

bool LaneArea::decide_lane_boundary_detect_type(void){
	for(int iNF = 0; iNF < CHX_NF_NUM; iNF++) {		// ////
		LaneRegion *pRegion = get_region(iNF);
		if(pRegion == NULL)	continue;
		// //
		for(int iLR = 0; iLR < CHX_LR_NUM; iLR++) {		// 
			LaneSide *pSide = pRegion->get_side(iLR);
			if(pSide == NULL)	continue;
			if(pSide->getFoundNowFlag() == true) {		// //
				pSide->incFoundCounter();				// 
				pSide->clearLostCounter();				// ////
			} else {
				pSide->clearFoundCounter();				// 
				pSide->incLostCounter();				// ////
			}
			if(pSide->getFoundFlag() == false) {		// //
				if(pSide->getFoundCounter() >= COUNT_FOR_FOUND_STATUS_LANE_SIDE) {	// ////
					pSide->setFoundFlag();				// ////
				}
			} else {									// ////
				if(pSide->getLostCounter() >= COUNT_FOR_LOST_STATUS_LANE_SIDE) {	// ////////
					pSide->clearFoundFlag();			// ////////
				}
			}
		}

		if(pRegion->get_side(CHX_LR_LEFT)->getFoundNowFlag() == true || pRegion->get_side(CHX_LR_RIGHT)->getFoundNowFlag() == true) {	// ////or
			pRegion->setFoundNowFlag();			// ////
		} else {
			pRegion->clearFoundNowFlag();		// //////////
		}
		if(pRegion->getFoundNowFlag() == true) {	// ////
			pRegion->incFoundCounter();				// 
			pRegion->clearLostCounter();			// ////
		} else {									// //////////
			pRegion->clearFoundCounter();			// 
			pRegion->incLostCounter();				// ////
		}
		if(pRegion->getFoundFlag() == false) {		// ////, //////
			if(pRegion->getFoundCounter() >= COUNT_FOR_FOUND_STATUS_LANE_REGION) {	// ////
				pRegion->setFoundFlag();		// ////
			}
		} else {									// //
			if(pRegion->getLostCounter() >= COUNT_FOR_LOST_STATUS_LANE_REGION) {	// ////////
				pRegion->clearFoundFlag();		// ////////
			}
		}
	}
	for(int iLR = 0; iLR < CHX_LR_NUM; iLR++) {
		switch(get_lane_boundary_detect_type(iLR)) {
		case	CHX_LBT_NONE:	// ///////
			{
				if(	0
					||	get_region(CHX_NF_NEAR)->get_side(iLR)->LaneBoundaryType(0) == CHX_LBT_LANEMARKER
					||	get_region(CHX_NF_FAR)->get_side(iLR)->LaneBoundaryType(0) == CHX_LBT_LANEMARKER) {	// //
						clearLaneBoundaryLostCounter(iLR);
						incLaneBoundaryFoundCounter(iLR);
						set_lane_boundary_detect_type(iLR, CHX_LBT_LANEMARKER);	// //
				} else	if(	0
					||	get_region(CHX_NF_NEAR)->get_side(iLR)->LaneBoundaryType(0) == CHX_LBT_LANEMARKERLINE
					||	get_region(CHX_NF_FAR)->get_side(iLR)->LaneBoundaryType(0) == CHX_LBT_LANEMARKERLINE) {	// 
						clearLaneBoundaryLostCounter(iLR);
						incLaneBoundaryFoundCounter(iLR);
						set_lane_boundary_detect_type(iLR, CHX_LBT_LANEMARKERLINE);	// //
				} else {															// ////
					clearLaneBoundaryFoundCounter(iLR);
					clearLaneBoundaryLostCounter(iLR);
				}
			}
			break;
		case	CHX_LBT_LANEMARKER:	//////
			{
				if(	0
					||	get_region(CHX_NF_NEAR)->get_side(iLR)->LaneBoundaryType(0) == CHX_LBT_LANEMARKER
					||	get_region(CHX_NF_FAR)->get_side(iLR)->LaneBoundaryType(0) == CHX_LBT_LANEMARKER) {		////
						clearLaneBoundaryLostCounter(iLR);
						incLaneBoundaryFoundCounter(iLR);
						if(LaneBoundaryFoundCounter(iLR) >= LBT_LANEMARKER_FOUND_COUNT) {		/////////
							set_lane_boundary_detect_type(iLR, CHX_LBT_LANEMARKER_TRACK);	///////
						} else {
						}
				} else	if(	0
					||	get_region(CHX_NF_NEAR)->get_side(iLR)->LaneBoundaryType(0) == CHX_LBT_LANEMARKERLINE
					||	get_region(CHX_NF_FAR)->get_side(iLR)->LaneBoundaryType(0) == CHX_LBT_LANEMARKERLINE) {	// 
						clearLaneBoundaryFoundCounter(iLR);
						clearLaneBoundaryLostCounter(iLR);
						set_lane_boundary_detect_type(iLR, CHX_LBT_NONE);	// ////
						getLaneParameterOneSide(iLR)->initialize();
				} else {		///////
					clearLaneBoundaryFoundCounter(iLR);
					clearLaneBoundaryLostCounter(iLR);
					set_lane_boundary_detect_type(iLR, CHX_LBT_NONE);	///////
					getLaneParameterOneSide(iLR)->initialize();
				}
			}
			break;
		case	CHX_LBT_LANEMARKERLINE:	//////
			{
				if(	0
					||	get_region(CHX_NF_NEAR)->get_side(iLR)->LaneBoundaryType(0) == CHX_LBT_LANEMARKER
					||	get_region(CHX_NF_FAR)->get_side(iLR)->LaneBoundaryType(0) == CHX_LBT_LANEMARKER) {	// //
						clearLaneBoundaryFoundCounter(iLR);
						clearLaneBoundaryLostCounter(iLR);
						incLaneBoundaryFoundCounter(iLR);
						set_lane_boundary_detect_type(iLR, CHX_LBT_LANEMARKER);	// //
				} else	if(	0
					||	get_region(CHX_NF_NEAR)->get_side(iLR)->LaneBoundaryType(0) == CHX_LBT_LANEMARKERLINE
					||	get_region(CHX_NF_FAR)->get_side(iLR)->LaneBoundaryType(0) == CHX_LBT_LANEMARKERLINE) {	// 
						clearLaneBoundaryLostCounter(iLR);
						incLaneBoundaryFoundCounter(iLR);
						if(LaneBoundaryFoundCounter(iLR) >= LBT_LANEMARKERLINE_FOUND_COUNT) {	/////////
							set_lane_boundary_detect_type(iLR, CHX_LBT_LANEMARKERLINE_TRACK);	////
						} else {
						}
				} else {										//////
					clearLaneBoundaryFoundCounter(iLR);
					clearLaneBoundaryLostCounter(iLR);
					set_lane_boundary_detect_type(iLR, CHX_LBT_NONE);	// ////
					getLaneParameterOneSide(iLR)->initialize();
				}
			}
			break;
		case	CHX_LBT_LANEMARKER_TRACK:	//////
			{
				int iLaneBoundaryPointNumberInNearArea = 0;
// 				if(getLaneBoundaryPointsInNearArea(iLR) != NULL) {
// 					iLaneBoundaryPointNumberInNearArea = getLaneBoundaryPointsInNearArea(iLR)->get_lane_marker_point_number();
// 				}
				if(	0
					||	get_region(CHX_NF_NEAR)->get_side(iLR)->LaneBoundaryType(0) == CHX_LBT_LANEMARKER
					||	get_region(CHX_NF_FAR)->get_side(iLR)->LaneBoundaryType(0) == CHX_LBT_LANEMARKER
					|| iLaneBoundaryPointNumberInNearArea > 0	// 20110925
					) {		/////
						clearLaneBoundaryLostCounter(iLR);
				} else {		///////
					clearLaneBoundaryFoundCounter(iLR);
					incLaneBoundaryLostCounter(iLR);
					if(LaneBoundaryLostCounter(iLR) >= LBT_LANEMARKER_LOST_COUNT) {		// //////////
						set_lane_boundary_detect_type(iLR, CHX_LBT_NONE);	//////
						getLaneParameterOneSide(iLR)->initialize();
					} else {
					}
				}
			}
			break;
		case	CHX_LBT_LANEMARKERLINE_TRACK:///////
			{
				if(	0
					||	get_region(CHX_NF_NEAR)->get_side(iLR)->LaneBoundaryType(0) == CHX_LBT_LANEMARKERLINE
					||	get_region(CHX_NF_FAR)->get_side(iLR)->LaneBoundaryType(0) == CHX_LBT_LANEMARKERLINE) {		// 
						clearLaneBoundaryLostCounter(iLR);
				} else {		// ////
					clearLaneBoundaryFoundCounter(iLR);
					incLaneBoundaryLostCounter(iLR);
					if(LaneBoundaryLostCounter(iLR) >= LBT_LANEMARKERLINE_LOST_COUNT) {		/////////////
						set_lane_boundary_detect_type(iLR, CHX_LBT_NONE);	////////
						getLaneParameterOneSide(iLR)->initialize();
					} else {
					}
				}
			}
			break;
		}
	}
	return true;
}

bool LaneArea::calculate_average_edge_strength_of_lane_marker_lines(void){
  //TODO:add your code here//
	for (int iNF = 0; iNF < CHX_NF_NUM; iNF++) {	////
		LaneRegion *pRegion = this->get_region(iNF);
		if (pRegion == NULL)
			continue;
		for (int iLR = 0; iLR < CHX_LR_NUM; iLR++) {	////
			LaneSide *pSide = pRegion->get_side(iLR);
			if (pSide == NULL)
				continue;
			for (int iUD = 0; iUD < CHX_UD_NUM; iUD++) {	/////
				LaneMarkerLines *pLMLs = pSide->get_lane_marker_lines(iUD);
				if (pLMLs == NULL)
					continue;
				for (int iIdx = pLMLs->get_lane_marker_line_number() - 1; iIdx >= 0;
						iIdx--) {	// //
					LaneMarkerLine *pLML = pLMLs->get_lane_marker_line(iIdx);
					if (pLML == NULL)
						continue;
					LaneMarkerPoints *pLMPs = pLML->get_lane_marker_points();
					if (pLMPs == NULL)
						continue;
					pLMPs->calculate_average_edge_strength();
				}
			}
		}
	}
	return true;
}


int calcNewThOfEdgeStrength(LaneMarkerPoints *pLMPs)
{
	if(pLMPs == NULL)	return 0;

	////////////
	pLMPs->get_average_of_edge_strength();
	/////
	double dNewTh = pLMPs->get_average_of_edge_strength() * AVERAGE_EDGE_STRENGTH_AND_TH_RATIO;

	//////
	if(dNewTh < 0 && dNewTh > CS4_NEGATIVETHRESHOLD)		{	dNewTh = CS4_NEGATIVETHRESHOLD;	}
	else if(dNewTh > 0 && dNewTh < CS4_POSITIVETHRESHOLD)	{	dNewTh = CS4_POSITIVETHRESHOLD;	}

	return (int)dNewTh;

}

bool LaneArea::adjust_edge_threshold(void){
	for (int iLR = 0; iLR < CHX_LR_NUM; iLR++) {
		LaneMarkerPoints *pLMPs = get_lane_boundary_points(iLR);
		int iNewTh = 0;
		if (pLMPs == NULL || pLMPs->get_lane_marker_point_number() == 0) {
			iNewTh = get_region(CHX_NF_NEAR)->get_side(iLR)->get_threshold(
					CHX_UD_UP) / 2;
			// //
			if (iNewTh < 0 && iNewTh > CS4_NEGATIVETHRESHOLD) {
				iNewTh = CS4_NEGATIVETHRESHOLD;
			} else if (iNewTh > 0 && iNewTh < CS4_POSITIVETHRESHOLD) {
				iNewTh = CS4_POSITIVETHRESHOLD;
			}
		} else {
			iNewTh = calcNewThOfEdgeStrength(pLMPs);
		}
		if (iNewTh == 0)
			continue;

		if (iNewTh < 0) {
			iNewTh = -iNewTh;
		}

		this->get_region(CHX_NF_NEAR)->get_side(iLR)->set_threshold(CHX_UD_UP,
				iNewTh);
		this->get_region(CHX_NF_NEAR)->get_side(iLR)->set_threshold(CHX_UD_DOWN,
				-iNewTh);
		this->get_region(CHX_NF_FAR)->get_side(iLR)->set_threshold(CHX_UD_UP, iNewTh);
		this->get_region(CHX_NF_FAR)->get_side(iLR)->set_threshold(CHX_UD_DOWN,
				-iNewTh);
	}
	return true;
}

bool LaneArea::calculate_process_line(CamParam *pCamParam, int iIstepNearArea, int iMaxLineNumberRegion0, int iMaxLineNumberRegion1, int iIstepFarArea){


	if(pCamParam == NULL)	return false;

		//////////
		int iIsrcTopRegion1 = calcHorizontalLineOfDepth(pCamParam, this->get_region(CHX_NF_FAR)->get_top());
		////////////
		if(iIsrcTopRegion1 <this->get_Isrc_min())	{	iIsrcTopRegion1 = this->get_Isrc_min();	}
		//////////
		double dZTopRegion1 = calcDepthOfHorizontalLine(pCamParam, iIsrcTopRegion1);
		while(dZTopRegion1 > this->get_region(CHX_NF_FAR)->get_top()) {
			//////
			iIsrcTopRegion1++;
			if(iIsrcTopRegion1 >= pCamParam->height)	return false;
			dZTopRegion1 = calcDepthOfHorizontalLine(pCamParam, iIsrcTopRegion1);
		}

		//////////
		int iIsrcBottomRegion1 = calcHorizontalLineOfDepth(pCamParam, this->get_region(CHX_NF_FAR)->get_bottom());
		if(iIsrcBottomRegion1 > this->get_Isrc_max())	{	iIsrcBottomRegion1 = this->get_Isrc_max();	}	/////
		//////////
		double dZBottomRegion1 = calcDepthOfHorizontalLine(pCamParam, iIsrcBottomRegion1);
		while(dZBottomRegion1 < get_region(CHX_NF_FAR)->get_bottom()) {
			iIsrcBottomRegion1--;
			if(iIsrcBottomRegion1 < 0)	return false;
			dZBottomRegion1 = calcDepthOfHorizontalLine(pCamParam, iIsrcBottomRegion1);
		}
		int iFullLineNumberRegion1 = iIsrcBottomRegion1 - iIsrcTopRegion1 + 1;
		/////////
		int iRealLineNumberRegion1 = iMaxLineNumberRegion1;		////////
		if(iRealLineNumberRegion1 > iFullLineNumberRegion1) {
			iRealLineNumberRegion1 = iFullLineNumberRegion1;
			////////////
		}

		//////////
		int iIsrcTopRegion0 = calcHorizontalLineOfDepth(pCamParam, this->get_region(CHX_NF_NEAR)->get_top());
		//////////
		double dZTopRegion0 = calcDepthOfHorizontalLine(pCamParam, iIsrcTopRegion0);
		if(iIsrcTopRegion0 < this->get_Isrc_min())	{	iIsrcTopRegion0 = this->get_Isrc_min();	}	////////
		while(dZTopRegion0 > this->get_region(CHX_NF_NEAR)->get_top()) {
			iIsrcTopRegion0++;
			if(iIsrcTopRegion0 >= pCamParam->height)	return false;
			dZTopRegion0 = calcDepthOfHorizontalLine(pCamParam, iIsrcTopRegion0);
		}
		////////
		int iIsrcBottomRegion0 = calcHorizontalLineOfDepth(pCamParam, this->get_region(CHX_NF_NEAR)->get_bottom());
		if(iIsrcBottomRegion0 > this->get_Isrc_max())	{	iIsrcBottomRegion0 = this->get_Isrc_max();	}	//////
		//////
		double dZBottomRegion0 = calcDepthOfHorizontalLine(pCamParam, iIsrcBottomRegion0);
		while(dZBottomRegion0 < this->get_region(CHX_NF_NEAR)->get_bottom()) {
			////////
			iIsrcBottomRegion0--;
			if(iIsrcBottomRegion0 < 0)	return false;
			dZBottomRegion1 = calcDepthOfHorizontalLine(pCamParam, iIsrcBottomRegion0);
		}

		int iFullLineNumberRegion0 = iIsrcBottomRegion0 - iIsrcTopRegion0 + 1;	//////
		int iRealLineNumberRegion0 = iMaxLineNumberRegion0;	/////
		if(iRealLineNumberRegion0 > iFullLineNumberRegion0) {
			iRealLineNumberRegion0 = iFullLineNumberRegion0;//////
		}

		////////
		//////////////
		int iIsrcTopFarArea=iIsrcTopRegion1-EXTENDED_HEIGHT_FOR_FAR_AREA;
		if(iIsrcTopFarArea > this->get_Isrc_min())	{	iIsrcTopFarArea = this->get_Isrc_min();	}	//////
		int iIsrcBottomFarArea = iIsrcTopRegion1 - 1;
		if(iIsrcBottomFarArea < this->get_Isrc_min())	{	iIsrcBottomFarArea = this->get_Isrc_min();	}	//////

		int iRealLineNumberFarArea = (iIsrcBottomFarArea - iIsrcTopFarArea) / iIstepFarArea + 1;	//////

		//////////
		int iIsrcTopNearArea = iIsrcBottomRegion0 + 1;	//////
		if(iIsrcTopNearArea > this->get_Isrc_max())	{	iIsrcTopNearArea = this->get_Isrc_max();	}	//////
		int iIsrcBottomNearArea = pCamParam->height - EXTENDED_HEIGHT_FOR_NEAR_AREA;	//////////
		if(iIsrcBottomNearArea > this->get_Isrc_max())	{	iIsrcBottomNearArea = this->get_Isrc_max();	}	//////

		int iRealLineNumberNearArea = (iIsrcBottomNearArea - iIsrcTopNearArea) / iIstepNearArea + 1;	////////////

		int iLineNumberAll = iRealLineNumberFarArea  + iRealLineNumberRegion1 + iRealLineNumberRegion0 + iRealLineNumberNearArea;	//////
		////////////////////////////////////
		int *piBuf = (int*)malloc(sizeof(int) * iLineNumberAll);
		if(piBuf == NULL)	{	this->set_process_line_number(0);	return false;	}
		this->set_process_line_number(iLineNumberAll);
		this->set_process_line(piBuf);

		//---------------FarArea---------------//
		int iSrcBase = 0;
		for(int iIdx = 0; iIdx < iRealLineNumberFarArea; iIdx++) {
			int iIsrc = iIsrcTopFarArea + iIstepFarArea * iIdx;
			(this->get_process_line())[iSrcBase + iIdx]	= iIsrc;
		}
		//-------------------Region1-----------------//
		iSrcBase = iRealLineNumberFarArea;
		(this->get_process_line())[iSrcBase]								= iIsrcTopRegion1;
		(this->get_process_line())[iSrcBase + iRealLineNumberRegion1 - 1]	= iIsrcBottomRegion1;

		//------------------------region1---------------------//
		double dInterval1 = (dZTopRegion1 - dZBottomRegion1) / (iRealLineNumberRegion1 - 1);
		for(int iIsrc = 1; iIsrc < iRealLineNumberRegion1 - 1; iIsrc++) {

			(this->get_process_line())[iSrcBase + iIsrc] = (this->get_process_line())[iSrcBase + iIsrc - 1] + 2;
			while(true) {
				////
				double dZ = calcDepthOfHorizontalLine(pCamParam, (this->get_process_line())[iSrcBase + iIsrc]);
				////
				//cout << "Dz" << dZ << endl;
				if(dZ < (dZTopRegion1 - dInterval1 * iIsrc)) {
					(this->get_process_line())[iSrcBase + iIsrc]--;
					break;
				}
				////
				(this->get_process_line())[iSrcBase + iIsrc]++;
			}
		}

		//-----------------Region0----------------//
		iSrcBase = iRealLineNumberFarArea + iRealLineNumberRegion1;
		(this->get_process_line())[iSrcBase]								= iIsrcTopRegion0;
		(this->get_process_line())[iSrcBase + iRealLineNumberRegion0 - 1]	= iIsrcBottomRegion0;
		double dInterval0 = (dZTopRegion0 - dZBottomRegion0) / (iRealLineNumberRegion0 - 1);
		for(int iIsrc = 1; iIsrc < iRealLineNumberRegion0 - 1; iIsrc++) {
			////
			(this->get_process_line())[iSrcBase + iIsrc] = (this->get_process_line())[iSrcBase + iIsrc - 1] + 2;
			while(true) {
				////
				double dZ = calcDepthOfHorizontalLine(pCamParam, (this->get_process_line())[iSrcBase + iIsrc]);
				////
				if(dZ < (dZTopRegion0 - dInterval0 * iIsrc)) {
					(this->get_process_line())[iSrcBase + iIsrc]--;
					break;
				}
				////
				(this->get_process_line())[iSrcBase + iIsrc]++;
			}
		}

		//----------------NearArea--------------------//
		iSrcBase = iRealLineNumberFarArea + iRealLineNumberRegion1 + iRealLineNumberRegion0;
		for(int iIdx = 0; iIdx < iRealLineNumberNearArea; iIdx++) {
			int iIsrc = iIsrcTopNearArea + iIstepNearArea * iIdx;
			(this->get_process_line())[iSrcBase + iIdx]	= iIsrc;
		}

		//////////////////////////////////////////////////
		//cout << "as" << endl;
		this->set_line_number_far_area(iRealLineNumberFarArea);
		this->set_line_number_near_area(iRealLineNumberNearArea);
		this->get_region(CHX_NF_FAR)->set_line_number(iRealLineNumberRegion1);
		this->get_region(CHX_NF_NEAR)->set_line_number(iRealLineNumberRegion0);

		return true;
}


bool LaneArea::decideDetectionStatus(CamParam *pCamParam)
{
	//TODO:add your code here//
	return true;
}
///////////////////////////////////////////////////////////////////
bool LaneArea::checkAndAdjustLaneParameters(void)	
{
	LaneParameterOneSide *pLPLeft = getLaneParameterOneSide(CHX_LR_LEFT);
	LaneParameterOneSide *pLPRight = getLaneParameterOneSide(CHX_LR_RIGHT);
	if(pLPLeft == NULL)	return false;
	if(pLPRight == NULL)	return false;

	for(int iLR = 0; iLR < CHX_LR_NUM; iLR++) {
		SearchInDefaultLanePositionFlag(iLR, false);
		LaneParameterOneSide *pLPOneSide = getLaneParameterOneSide(iLR);
		if(pLPOneSide == NULL)	continue;
		if(pLPOneSide->Available() == false)	continue;
		int iK = pLPOneSide->K();
		double dOffset = pLPOneSide->Param(CHX_LPID_OFFSET);
		if(fabs(dOffset)> fabs(pLPOneSide->ParamInit(CHX_LPID_OFFSET) * FACTOR_FOR_SEARCHING_IN_DEFAULT_LANE_POSITION * iK)) {	// 20111202	//
			SearchInDefaultLanePositionFlag(iLR, true);
		}
	}

	if(	get_lane_boundary_detect_type(CHX_LR_LEFT) >= CHX_LBT_LANEMARKERLINE_TRACK
		&&	get_lane_boundary_detect_type(CHX_LR_RIGHT) >= CHX_LBT_LANEMARKERLINE_TRACK
		) {
			double dLaneWidth	=	pLPRight->Param(CHX_LPID_OFFSET) - pLPLeft->Param(CHX_LPID_OFFSET);
			if(dLaneWidth < DB_LW_MIN) {
				if(fabs(pLPLeft->Param(CHX_LPID_OFFSET)) < fabs(pLPRight->Param(CHX_LPID_OFFSET))) {
					pLPLeft->initialize();
					set_lane_boundary_detect_type(CHX_LR_LEFT, CHX_LBT_NONE);
				} else {
					pLPRight->initialize();
					set_lane_boundary_detect_type(CHX_LR_RIGHT, CHX_LBT_NONE);
				}
			}
	}


	if(	get_lane_boundary_detect_type(CHX_LR_LEFT) >= CHX_LBT_LANEMARKERLINE_TRACK
		&&	get_lane_boundary_detect_type(CHX_LR_RIGHT) >= CHX_LBT_LANEMARKERLINE_TRACK
		) {
			double dX_NearTopL = pLPLeft->LaneBoundaryPositionOnRoad(-1, get_region(CHX_NF_NEAR)->get_top());
			double dX_NearTopR = pLPRight->LaneBoundaryPositionOnRoad(1, get_region(CHX_NF_NEAR)->get_top());
			double dLaneWidth_NearTopL	=	dX_NearTopR - dX_NearTopL;
			if(dLaneWidth_NearTopL < DB_LW_MIN) {
				if(fabs(dX_NearTopL) < fabs(dX_NearTopR)) {
					pLPLeft->initialize();
					set_lane_boundary_detect_type(CHX_LR_LEFT, CHX_LBT_NONE);
				} else {
					pLPRight->initialize();
					set_lane_boundary_detect_type(CHX_LR_RIGHT, CHX_LBT_NONE);
				}
			}
	}

	// lane change//
	if(	(	pLPLeft->Available() == true	&&	(pLPLeft->Param(CHX_LPID_OFFSET) > 0))
		||	(	pLPRight->Available() == true	&&	(pLPRight->Param(CHX_LPID_OFFSET) < 0))
		)	{
			for(int iLR = 0; iLR < CHX_LR_NUM; iLR++) {
				getLaneParameterOneSide(iLR)->initialize();
				set_lane_boundary_detect_type(iLR, CHX_LBT_NONE);
			}
			return true;
	}
	// yaw//
	// right branch//
	if(	pLPLeft->Available() == true
		&&	(fabs(pLPLeft->Param(CHX_LPID_THETA)) < MAXIMUM_YAW_FOR_MAIN_LANE)
		&&	pLPRight->Available() == true
		&&	(fabs(pLPRight->Param(CHX_LPID_THETA)) > MINIMUM_YAW_FOR_BRANCH_LANE)
		) {
			pLPRight->Param(CHX_LPID_THETA, pLPLeft->Param(CHX_LPID_THETA));
	}
	// left branch//
	if(	pLPRight->Available() == true
		&&	(fabs(pLPRight->Param(CHX_LPID_THETA)) < MAXIMUM_YAW_FOR_MAIN_LANE)
		&&	pLPLeft->Available() == true
		&&	(fabs(pLPLeft->Param(CHX_LPID_THETA)) > MINIMUM_YAW_FOR_BRANCH_LANE)
		) {
			pLPLeft->Param(CHX_LPID_THETA, pLPRight->Param(CHX_LPID_THETA));
	}

	return true;
}

bool LaneArea::removeIsolatedLaneBoundaryPoints(CamParam *pCamParam, int iLR)
{
	if(pCamParam == NULL)	return false;
	LaneMarkerPoints *pLMPs = get_lane_boundary_points(iLR);
	if(pLMPs == NULL)	return false;

	int iPointNum = pLMPs->get_lane_marker_point_number();
	int iIdxMax = iPointNum - 1;
	double Zvehicle=0.0,ZvehicleNext=0.0;
	for(int iIdx = 0; iIdx < iPointNum - 1; iIdx++) {
		LaneMarkerPoint *pLMP = pLMPs->get_lane_marker_point(iIdx);
		LaneMarkerPoint *pLMPNext = pLMPs->get_lane_marker_point(iIdx + 1);
		if((pLMP == NULL)||(pLMPNext==NULL))	continue;

		Zvehicle=calcZvehicleFromIsrc(pCamParam,pLMP->get_Isrc());
		ZvehicleNext=calcZvehicleFromIsrc(pCamParam,pLMPNext->get_Isrc());

		if(Zvehicle >= this->get_region(CHX_NF_FAR)->get_top()) continue;
		if((Zvehicle - ZvehicleNext) > MAXINUM_GAP_FOR_ISOLATED_LANE_BOUNDARY) {
			iIdxMax = iIdx;
			break;
		}
	}
	for(int iIdx = iPointNum - 1; iIdx > iIdxMax; iIdx--) {
		pLMPs->delete_lane_marker_point(iIdx);
	}

	return true;
}