#include "data.h"
#include "SatellitePositionAndClockError.h"

SatellitePositionAndClockError::SatellitePositionAndClockError()
{
}

//Compute Satellite Position, Velocity, and Clock Error
std::vector<SATMIDRES> SatellitePositionAndClockError::CalculatePVCE(const EpoRange& range, const std::vector<GPSEPHREC>& GPSEph, const std::vector<BDSEPHREC>& BDSEph, const int& index)
{
    SatMidRes.clear();
    if (range.SatNum == 0)
    {
        std::cout << "No available satellites!" << std::endl;
        return SatMidRes;
    }
    SatMidRes.reserve(range.SatNum);

    double t_oe = 0;
    for (int i = 0; i < range.SatNum; ++i)
    {
        //Time Correction
        double t = 0;
        if (index == 1)
        {
            t = range.Time.gpsSeconds - range.ComObs[i].PIF / C_Light;
        }
        else if (index == 2)
        {
            t = range.Time.gpsSeconds - range.ComObs[i].PIF / C_Light - range.SatMidRes[i].SatClkOft;
        }

        if (range.satRange[i].system == GPS)
        {
            if (GPSEph.size() == 0)
            {
                SATMIDRES SatMidResTemp;
                SatMidResTemp.t = t;
                SatMidRes.push_back(SatMidResTemp);
                continue;
            }
            t_oe = calculateToe(range.satRange[i], GPSEph, t);
            
            //Check if Ephemeris is Outdated
            if (abs(t - t_oe) > 7500)
            {
                SATMIDRES SatMidResTemp;
                SatMidResTemp.t = t;
                SatMidRes.push_back(SatMidResTemp);
                continue;
            }

            calculate(range.satRange[i], GPSEph, t, t_oe);
        }
        else
        {
            if (BDSEph.size() == 0)
            {
                SATMIDRES SatMidResTemp;
                SatMidResTemp.t = t;
                SatMidRes.push_back(SatMidResTemp);
                continue;
            }
            t_oe = calculateToe(range.satRange[i], BDSEph, t);
            
            //Check if Ephemeris is Outdated
            if (abs(t - t_oe) > 3900)
            {
                SATMIDRES SatMidResTemp;
                SatMidResTemp.t = t;
                SatMidRes.push_back(SatMidResTemp);
                continue;
            }

            double t_temp = t;
            calculate(range.satRange[i], BDSEph, t_temp, t_oe);
        }
    }

    return SatMidRes;
}

//Find the Nearest TOE
template <typename EPHCAL>
double SatellitePositionAndClockError::calculateToe(const SatRange& satRange, const EPHCAL& Eph_cal, const double& t)
{
    int PRNnum = 0;
    double t_oe = 0.0;
    for (int j = 0; j < Eph_cal.size(); ++j)
    {
        int closestBaseTime;
        int closestDistance;

        if ((satRange.PRN == Eph_cal[j].PRN) && (PRNnum == 0))
        {
            closestBaseTime = Eph_cal[j].TOE;
            closestDistance = abs(t - Eph_cal[j].TOE);
            while (closestDistance > 302400)
            {
                closestDistance = closestDistance - 604800;
            }
            while (closestDistance < -302400)
            {
                closestDistance = closestDistance + 604800;
            }
            PRNnum++;
        }
        else if ((satRange.PRN == Eph_cal[j].PRN) && (PRNnum != 0))
        {
            int distance = abs(t - Eph_cal[j].TOE);
            while (distance > 302400)
            {
                distance = distance - 604800;
            }
            while (distance < -302400)
            {
                distance = distance + 604800;
            }
            if (distance < closestDistance)
            {
                closestBaseTime = Eph_cal[j].TOE;
                closestDistance = distance;
            }
        }
        else
        {
            continue;
        }
        t_oe = closestBaseTime;
        PRNnum = 0;
    }

    //Unable to find TOE, Output Warning
    if (t_oe == 0.0)
    {
        char sys;
        if (std::is_same_v<EPHCAL, std::vector<GPSEPHREC>>)
        {
            sys = 'G';
        }
        else
        {
            sys = 'C';
        }
        std::cout << "Unable to find TOE! PRN: " << sys << satRange.PRN << std::endl;
    }

    return t_oe;
}

//Function for Calculation
template <typename EPHCAL>
void SatellitePositionAndClockError::calculate(const SatRange& satRange, const EPHCAL& Eph_cal, double t, const double& t_oe)
{
    SATMIDRES SatMidResTemp;
    SatMidResTemp.t = t;
    bool weak = true;
    for (int j = 0; j < Eph_cal.size(); ++j)
    {
        //Find the Relevant Ephemeris
        if ((t_oe == Eph_cal[j].TOE) && (satRange.PRN == Eph_cal[j].PRN) && (Eph_cal[j].SVHealth == 0))
        {
            weak = false;
            double A, n_0;
            if (std::is_same_v<EPHCAL, std::vector<GPSEPHREC>>)
            {
                A = Eph_cal[j].SqrtA;
                n_0 = sqrt(GPS_GM / (A * A * A));
            }
            else
            {
                A = Eph_cal[j].SqrtA * Eph_cal[j].SqrtA;
                n_0 = sqrt(BDS_GM / (A * A * A));
                t = t - 14;
            }

            double t_k = t - t_oe;
            while (t_k > 302400)
            {
                t_k = t_k - 604800;
            }
            while (t_k < -302400)
            {
                t_k = t_k + 604800;
            }
            double n = n_0 + Eph_cal[j].DeltaN;
            double M_k = Eph_cal[j].M0 + n * t_k;
            double E0 = M_k;
            double E1 = M_k + Eph_cal[j].e * sin(E0);
            while (fabs(E1 - E0) > 1e-14)
            {
                E0 = E1;
                E1 = M_k + Eph_cal[j].e * sin(E0);
            }
            double E_k = E1;
            double v_k = atan2((sqrt(1 - Eph_cal[j].e * Eph_cal[j].e) * sin(E_k)), (cos(E_k) - Eph_cal[j].e));
            double phi_k = v_k + Eph_cal[j].omega;
            double delta_u_k = Eph_cal[j].Cus * sin(2 * phi_k) + Eph_cal[j].Cuc * cos(2 * phi_k);
            double delta_r_k = Eph_cal[j].Crs * sin(2 * phi_k) + Eph_cal[j].Crc * cos(2 * phi_k);
            double delta_i_k = Eph_cal[j].Cis * sin(2 * phi_k) + Eph_cal[j].Cic * cos(2 * phi_k);
            double u_k = phi_k + delta_u_k;
            double r_k = A * (1 - Eph_cal[j].e * cos(E_k)) + delta_r_k;
            double i_k = Eph_cal[j].i0 + delta_i_k + Eph_cal[j].iDot * t_k;
            double x_k_ = r_k * cos(u_k);
            double y_k_ = r_k * sin(u_k);
            double OMEGA_k;

            if (std::is_same_v<EPHCAL, std::vector<GPSEPHREC>>)
            {
                OMEGA_k = Eph_cal[j].OMEGA + (Eph_cal[j].OMEGADot - GPS_OMEGA_DOT_e) * t_k - GPS_OMEGA_DOT_e * t_oe;
            }
            else
            {
                if (Eph_cal[j].i0 < (PI / 6.0))
                {
                    OMEGA_k = Eph_cal[j].OMEGA + Eph_cal[j].OMEGADot * t_k - BDS_OMEGA_DOT_e * t_oe;
                }
                else
                {
                    OMEGA_k = Eph_cal[j].OMEGA + (Eph_cal[j].OMEGADot - BDS_OMEGA_DOT_e) * t_k - BDS_OMEGA_DOT_e * t_oe;
                }
            }

            double x_k = x_k_ * cos(OMEGA_k) - y_k_ * cos(i_k) * sin(OMEGA_k);
            double y_k = x_k_ * sin(OMEGA_k) + y_k_ * cos(i_k) * cos(OMEGA_k);
            double z_k = y_k_ * sin(i_k);

            Matrix R_x(3, 3), R_z(3, 3), X_k(3, 1), X_GK(3, 1);
            Matrix mid(3, 3);

            if ((std::is_same_v<EPHCAL, std::vector<BDSEPHREC>>) && (Eph_cal[j].i0 < (PI / 6.0)))
            {
                R_x(0, 0) = 1;
                R_x(1, 1) = cos(-5 / 180.0 * PI);
                R_x(1, 2) = sin(-5 / 180.0 * PI);
                R_x(2, 1) = -sin(-5 / 180.0 * PI);
                R_x(2, 2) = cos(-5 / 180.0 * PI);
                R_z(0, 0) = cos(BDS_OMEGA_DOT_e * t_k);
                R_z(0, 1) = sin(BDS_OMEGA_DOT_e * t_k);
                R_z(1, 0) = -sin(BDS_OMEGA_DOT_e * t_k);
                R_z(1, 1) = cos(BDS_OMEGA_DOT_e * t_k);
                R_z(2, 2) = 1;
                X_GK(0, 0) = x_k;
                X_GK(1, 0) = y_k;
                X_GK(2, 0) = z_k;
                mid.MatrixMultiplication(R_z, R_x);
                X_k.MatrixMultiplication(mid, X_GK);
                x_k = X_k(0, 0);
                y_k = X_k(1, 0);
                z_k = X_k(2, 0);
            }

            //Satellite Position Results
            SatMidResTemp.SatPos[0] = x_k;
            SatMidResTemp.SatPos[1] = y_k;
            SatMidResTemp.SatPos[2] = z_k;

            double dE_k = n / (1 - Eph_cal[j].e * cos(E_k));
            double delta_t_r = F_ * Eph_cal[j].e * sqrt(A) * sin(E_k);
            double delta_t_r_dot = F_ * Eph_cal[j].e * sqrt(A) * cos(E_k) * dE_k;

            //Clock Error Results
            SatMidResTemp.SatClkOft = Eph_cal[j].ClkBias + Eph_cal[j].ClkDrift * (t - Eph_cal[j].TOC) + Eph_cal[j].ClkDriftRate * (t - Eph_cal[j].TOC) * (t - Eph_cal[j].TOC) + delta_t_r;
            SatMidResTemp.SatClkSft = Eph_cal[j].ClkDrift + 2 * Eph_cal[j].ClkDriftRate * (t - Eph_cal[j].TOC) + delta_t_r_dot;

            double dphi_k = sqrt((1 + Eph_cal[j].e) / (1 - Eph_cal[j].e)) * pow((cos(v_k / 2.0)) / (cos(E_k / 2.0)), 2) * dE_k;
            double du_k = 2 * (Eph_cal[j].Cus * cos(2 * phi_k) - Eph_cal[j].Cuc * sin(2 * phi_k)) * dphi_k + dphi_k;
            double dr_k = A * Eph_cal[j].e * sin(E_k) * dE_k + 2 * (Eph_cal[j].Crs * cos(2 * phi_k) - Eph_cal[j].Crc * sin(2 * phi_k)) * dphi_k;
            double dI_k = Eph_cal[j].iDot + 2 * (Eph_cal[j].Cis * cos(2 * phi_k) - Eph_cal[j].Cic * sin(2 * phi_k)) * dphi_k;
            double dOMEGA_k;

            Matrix RDot(3, 4);
            RDot(0, 0) = cos(OMEGA_k);
            RDot(0, 1) = -sin(OMEGA_k) * cos(i_k);
            RDot(0, 2) = -(x_k_ * sin(OMEGA_k) + y_k_ * cos(OMEGA_k) * cos(i_k));
            RDot(0, 3) = y_k_ * sin(OMEGA_k) * sin(i_k);
            RDot(1, 0) = sin(OMEGA_k);
            RDot(1, 1) = cos(OMEGA_k) * cos(i_k);
            RDot(1, 2) = x_k_ * cos(OMEGA_k) - y_k_ * sin(OMEGA_k) * cos(i_k);
            RDot(1, 3) = -y_k_ * cos(OMEGA_k) * sin(i_k);
            RDot(2, 1) = sin(i_k);
            RDot(2, 3) = y_k_ * cos(i_k);

            double x_k_Dot = dr_k * cos(u_k) - r_k * du_k * sin(u_k);
            double y_k_Dot = dr_k * sin(u_k) + r_k * du_k * cos(u_k);

            Matrix dX_k(3, 1), dX_k_(4, 1);
            dX_k_(0, 0) = x_k_Dot;
            dX_k_(1, 0) = y_k_Dot;
            dX_k_(3, 0) = dI_k;

            if (std::is_same_v<EPHCAL, std::vector<GPSEPHREC>>)
            {
                dOMEGA_k = Eph_cal[j].OMEGADot - GPS_OMEGA_DOT_e;
            }
            else
            {
                if (Eph_cal[j].i0 < (PI / 6.0))
                {
                    dOMEGA_k = Eph_cal[j].OMEGADot;
                }
                else
                {
                    dOMEGA_k = Eph_cal[j].OMEGADot - BDS_OMEGA_DOT_e;
                }
            }

            dX_k_(2, 0) = dOMEGA_k;
            dX_k.MatrixMultiplication(RDot, dX_k_);

            if ((std::is_same_v<EPHCAL, std::vector<BDSEPHREC>>) && (Eph_cal[j].i0 < (PI / 6.0)))
            {
                Matrix dX_k_mid(3, 1), dR_z(3, 3);
                dX_k_mid.MatrixMultiplication(mid, dX_k);
                dR_z(0, 0) = -sin(BDS_OMEGA_DOT_e * t_k) * BDS_OMEGA_DOT_e;
                dR_z(0, 1) = cos(BDS_OMEGA_DOT_e * t_k) * BDS_OMEGA_DOT_e;
                dR_z(1, 0) = -cos(BDS_OMEGA_DOT_e * t_k) * BDS_OMEGA_DOT_e;
                dR_z(1, 1) = -sin(BDS_OMEGA_DOT_e * t_k) * BDS_OMEGA_DOT_e;
                Matrix dmid1(3, 3), dmid2(3, 1);
                dmid1.MatrixMultiplication(dR_z, R_x);
                dmid2.MatrixMultiplication(dmid1, X_GK);
                dX_k.MatrixAddition(dX_k_mid, dmid2);
            }

            //Satellite Velocity Results
            for (int p = 0; p < 3; ++p)
            {
                SatMidResTemp.SatVel[p] = dX_k(p, 0);
            }

            SatMidResTemp.Valid = true;
            SatMidRes.push_back(SatMidResTemp);
            break;
        }
    }

    //Unable to Find Ephemeris for the Satellite, Output Warning
    if (weak)
    {
        char sys;
        if (std::is_same_v<EPHCAL, std::vector<GPSEPHREC>>)
        {
            sys = 'G';
        }
        else
        {
            sys = 'C';
        }
        //std::cout << "Unable to find ephemeris for the satellite, PRN: " << sys << satRange.PRN << std::endl;
        SatMidResTemp.t = t;
        SatMidResTemp.SatPos[0] = 0;
        SatMidResTemp.SatPos[1] = 0;
        SatMidResTemp.SatPos[2] = 0;
        SatMidRes.push_back(SatMidResTemp);
    }
}
