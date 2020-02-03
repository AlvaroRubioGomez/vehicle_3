function VerticalLoads = LoadTransferModel(z,phi,mu,z__dot,phi__dot,mu__dot,Omega,u)

    % ----------------------------------------------------------------
    %% Function purpose: compute vertical loads for the 4 wheels 
    % ----------------------------------------------------------------

    params = getVehicleDataStruct;
    Ks__r = params.rear_suspension.Ks_r;
    Ks__f = params.front_suspension.Ks_f;
    Cs__r__b = params.rear_suspension.Cs_r_b;
    Cs__r__r = params.rear_suspension.Cs_r_r;
    Cs__f__b = params.front_suspension.Cs_f_b;
    Cs__f__r = params.front_suspension.Cs_f_r;
    Karb_r = params.rear_suspension.Karb_r;
    Karb_f = params.front_suspension.Karb_f;
    stroke__r = params.rear_suspension.stroke_r;
    K__es_r = params.rear_suspension.K_es_r;
    C__es_r = params.rear_suspension.C_es_r;
    h__rc_r = params.rear_suspension.h_rc_r;
    stroke__f = params.front_suspension.stroke_f;
    K__es_f = params.front_suspension.K_es_f;
    C__es_f = params.front_suspension.C_es_f;
    h__rc_f = params.front_suspension.h_rc_f;
    z__rlx_r = params.rear_suspension.z__rlx_r;
    z__rlx_f = params.front_suspension.z__rlx_f;

    reg_fact = params.rear_suspension.reg_fact;
    Lr = params.vehicle.Lr;
    Lf = params.vehicle.Lf;
    Wr = params.vehicle.Wr;
    Wf = params.vehicle.Wf;
    ms = params.vehicle.ms;
    m = params.vehicle.m;
    g = params.vehicle.g;

    Fz__rr = -0.5e0 * K__es_r * (reg_fact * (-h__rc_r + Lr * mu + Lr * (h__rc_r - h__rc_f) / (Lf + Lr) - phi * Wr / 0.2e1 + z - stroke__r / 0.2e1) * (0.1e1 + reg_fact ^ 2 * (-h__rc_r + Lr * mu + Lr * (h__rc_r - h__rc_f) / (Lf + Lr) - phi * Wr / 0.2e1 + z - stroke__r / 0.2e1) ^ 2) ^ (-0.1e1 / 0.2e1) + 0.1e1) * (-h__rc_r + Lr * mu + Lr * (h__rc_r - h__rc_f) / (Lf + Lr) - phi * Wr / 0.2e1 + z - stroke__r / 0.2e1) - 0.5e0 * C__es_r * (reg_fact * (-h__rc_r + Lr * mu + Lr * (h__rc_r - h__rc_f) / (Lf + Lr) - phi * Wr / 0.2e1 + z - stroke__r / 0.2e1) * (0.1e1 + reg_fact ^ 2 * (-h__rc_r + Lr * mu + Lr * (h__rc_r - h__rc_f) / (Lf + Lr) - phi * Wr / 0.2e1 + z - stroke__r / 0.2e1) ^ 2) ^ (-0.1e1 / 0.2e1) + 0.1e1) * (Lr * mu__dot - phi__dot * Wr / 0.2e1 + z__dot) + m * g * Lf / (0.2e1 * Lf + 0.2e1 * Lr) + m * Omega * u * Lf / (Lf + Lr) * h__rc_r / Wr + Ks__r * (h__rc_r - Lr * mu - Lr * (h__rc_r - h__rc_f) / (Lf + Lr) + phi * Wr / 0.2e1 - z) + 0.5e0 * K__es_r * (reg_fact * (h__rc_r - Lr * mu - Lr * (h__rc_r - h__rc_f) / (Lf + Lr) + phi * Wr / 0.2e1 - z - stroke__r / 0.2e1) * (0.1e1 + reg_fact ^ 2 * (h__rc_r - Lr * mu - Lr * (h__rc_r - h__rc_f) / (Lf + Lr) + phi * Wr / 0.2e1 - z - stroke__r / 0.2e1) ^ 2) ^ (-0.1e1 / 0.2e1) + 0.1e1) * (h__rc_r - Lr * mu - Lr * (h__rc_r - h__rc_f) / (Lf + Lr) + phi * Wr / 0.2e1 - z - stroke__r / 0.2e1) + 0.5e0 * C__es_r * (reg_fact * (h__rc_r - Lr * mu - Lr * (h__rc_r - h__rc_f) / (Lf + Lr) + phi * Wr / 0.2e1 - z - stroke__r / 0.2e1) * (0.1e1 + reg_fact ^ 2 * (h__rc_r - Lr * mu - Lr * (h__rc_r - h__rc_f) / (Lf + Lr) + phi * Wr / 0.2e1 - z - stroke__r / 0.2e1) ^ 2) ^ (-0.1e1 / 0.2e1) + 0.1e1) * (-Lr * mu__dot + phi__dot * Wr / 0.2e1 - z__dot) - 0.5e0 * Cs__r__r * (reg_fact * (-Lr * mu__dot + phi__dot * Wr / 0.2e1 - z__dot) * (0.1e1 + reg_fact ^ 2 * (-Lr * mu__dot + phi__dot * Wr / 0.2e1 - z__dot) ^ 2) ^ (-0.1e1 / 0.2e1) - 0.1e1) * (-Lr * mu__dot + phi__dot * Wr / 0.2e1 - z__dot) + 0.5e0 * Cs__r__b * (reg_fact * (-Lr * mu__dot + phi__dot * Wr / 0.2e1 - z__dot) * (0.1e1 + reg_fact ^ 2 * (-Lr * mu__dot + phi__dot * Wr / 0.2e1 - z__dot) ^ 2) ^ (-0.1e1 / 0.2e1) + 0.1e1) * (-Lr * mu__dot + phi__dot * Wr / 0.2e1 - z__dot) + Karb_r * phi * Wr;
    Fz__rl = -0.5e0 * K__es_r * (reg_fact * (-h__rc_r + Lr * mu + Lr * (h__rc_r - h__rc_f) / (Lf + Lr) + phi * Wr / 0.2e1 + z - stroke__r / 0.2e1) * (0.1e1 + reg_fact ^ 2 * (-h__rc_r + Lr * mu + Lr * (h__rc_r - h__rc_f) / (Lf + Lr) + phi * Wr / 0.2e1 + z - stroke__r / 0.2e1) ^ 2) ^ (-0.1e1 / 0.2e1) + 0.1e1) * (-h__rc_r + Lr * mu + Lr * (h__rc_r - h__rc_f) / (Lf + Lr) + phi * Wr / 0.2e1 + z - stroke__r / 0.2e1) - 0.5e0 * C__es_r * (reg_fact * (-h__rc_r + Lr * mu + Lr * (h__rc_r - h__rc_f) / (Lf + Lr) + phi * Wr / 0.2e1 + z - stroke__r / 0.2e1) * (0.1e1 + reg_fact ^ 2 * (-h__rc_r + Lr * mu + Lr * (h__rc_r - h__rc_f) / (Lf + Lr) + phi * Wr / 0.2e1 + z - stroke__r / 0.2e1) ^ 2) ^ (-0.1e1 / 0.2e1) + 0.1e1) * (Lr * mu__dot + phi__dot * Wr / 0.2e1 + z__dot) + m * g * Lf / (0.2e1 * Lf + 0.2e1 * Lr) - m * Omega * u * Lf / (Lf + Lr) * h__rc_r / Wr + Ks__r * (h__rc_r - Lr * mu - Lr * (h__rc_r - h__rc_f) / (Lf + Lr) - phi * Wr / 0.2e1 - z) + 0.5e0 * K__es_r * (reg_fact * (h__rc_r - Lr * mu - Lr * (h__rc_r - h__rc_f) / (Lf + Lr) - phi * Wr / 0.2e1 - z - stroke__r / 0.2e1) * (0.1e1 + reg_fact ^ 2 * (h__rc_r - Lr * mu - Lr * (h__rc_r - h__rc_f) / (Lf + Lr) - phi * Wr / 0.2e1 - z - stroke__r / 0.2e1) ^ 2) ^ (-0.1e1 / 0.2e1) + 0.1e1) * (h__rc_r - Lr * mu - Lr * (h__rc_r - h__rc_f) / (Lf + Lr) - phi * Wr / 0.2e1 - z - stroke__r / 0.2e1) + 0.5e0 * C__es_r * (reg_fact * (h__rc_r - Lr * mu - Lr * (h__rc_r - h__rc_f) / (Lf + Lr) - phi * Wr / 0.2e1 - z - stroke__r / 0.2e1) * (0.1e1 + reg_fact ^ 2 * (h__rc_r - Lr * mu - Lr * (h__rc_r - h__rc_f) / (Lf + Lr) - phi * Wr / 0.2e1 - z - stroke__r / 0.2e1) ^ 2) ^ (-0.1e1 / 0.2e1) + 0.1e1) * (-Lr * mu__dot - phi__dot * Wr / 0.2e1 - z__dot) - 0.5e0 * Cs__r__r * (reg_fact * (-Lr * mu__dot - phi__dot * Wr / 0.2e1 - z__dot) * (0.1e1 + reg_fact ^ 2 * (-Lr * mu__dot - phi__dot * Wr / 0.2e1 - z__dot) ^ 2) ^ (-0.1e1 / 0.2e1) - 0.1e1) * (-Lr * mu__dot - phi__dot * Wr / 0.2e1 - z__dot) + 0.5e0 * Cs__r__b * (reg_fact * (-Lr * mu__dot - phi__dot * Wr / 0.2e1 - z__dot) * (0.1e1 + reg_fact ^ 2 * (-Lr * mu__dot - phi__dot * Wr / 0.2e1 - z__dot) ^ 2) ^ (-0.1e1 / 0.2e1) + 0.1e1) * (-Lr * mu__dot - phi__dot * Wr / 0.2e1 - z__dot) - Karb_r * phi * Wr;
    Fz__fr = -0.5e0 * K__es_f * (reg_fact * (-h__rc_f - Lf * mu - Lf * (h__rc_r - h__rc_f) / (Lf + Lr) - phi * Wf / 0.2e1 + z - stroke__f / 0.2e1) * (0.1e1 + reg_fact ^ 2 * (-h__rc_f - Lf * mu - Lf * (h__rc_r - h__rc_f) / (Lf + Lr) - phi * Wf / 0.2e1 + z - stroke__f / 0.2e1) ^ 2) ^ (-0.1e1 / 0.2e1) + 0.1e1) * (-h__rc_f - Lf * mu - Lf * (h__rc_r - h__rc_f) / (Lf + Lr) - phi * Wf / 0.2e1 + z - stroke__f / 0.2e1) - 0.5e0 * C__es_f * (reg_fact * (-h__rc_f - Lf * mu - Lf * (h__rc_r - h__rc_f) / (Lf + Lr) - phi * Wf / 0.2e1 + z - stroke__f / 0.2e1) * (0.1e1 + reg_fact ^ 2 * (-h__rc_f - Lf * mu - Lf * (h__rc_r - h__rc_f) / (Lf + Lr) - phi * Wf / 0.2e1 + z - stroke__f / 0.2e1) ^ 2) ^ (-0.1e1 / 0.2e1) + 0.1e1) * (-Lf * mu__dot - phi__dot * Wf / 0.2e1 + z__dot) + m * g * Lr / (0.2e1 * Lf + 0.2e1 * Lr) + m * Omega * u * Lr / (Lf + Lr) * h__rc_f / Wf + Ks__f * (h__rc_f + Lf * mu + Lf * (h__rc_r - h__rc_f) / (Lf + Lr) + phi * Wf / 0.2e1 - z) + 0.5e0 * K__es_f * (reg_fact * (h__rc_f + Lf * mu + Lf * (h__rc_r - h__rc_f) / (Lf + Lr) + phi * Wf / 0.2e1 - z - stroke__f / 0.2e1) * (0.1e1 + reg_fact ^ 2 * (h__rc_f + Lf * mu + Lf * (h__rc_r - h__rc_f) / (Lf + Lr) + phi * Wf / 0.2e1 - z - stroke__f / 0.2e1) ^ 2) ^ (-0.1e1 / 0.2e1) + 0.1e1) * (h__rc_f + Lf * mu + Lf * (h__rc_r - h__rc_f) / (Lf + Lr) + phi * Wf / 0.2e1 - z - stroke__f / 0.2e1) + 0.5e0 * C__es_f * (reg_fact * (h__rc_f + Lf * mu + Lf * (h__rc_r - h__rc_f) / (Lf + Lr) + phi * Wf / 0.2e1 - z - stroke__f / 0.2e1) * (0.1e1 + reg_fact ^ 2 * (h__rc_f + Lf * mu + Lf * (h__rc_r - h__rc_f) / (Lf + Lr) + phi * Wf / 0.2e1 - z - stroke__f / 0.2e1) ^ 2) ^ (-0.1e1 / 0.2e1) + 0.1e1) * (Lf * mu__dot + phi__dot * Wf / 0.2e1 - z__dot) - 0.5e0 * Cs__f__r * (reg_fact * (Lf * mu__dot + phi__dot * Wf / 0.2e1 - z__dot) * (0.1e1 + reg_fact ^ 2 * (Lf * mu__dot + phi__dot * Wf / 0.2e1 - z__dot) ^ 2) ^ (-0.1e1 / 0.2e1) - 0.1e1) * (Lf * mu__dot + phi__dot * Wf / 0.2e1 - z__dot) + 0.5e0 * Cs__f__b * (reg_fact * (Lf * mu__dot + phi__dot * Wf / 0.2e1 - z__dot) * (0.1e1 + reg_fact ^ 2 * (Lf * mu__dot + phi__dot * Wf / 0.2e1 - z__dot) ^ 2) ^ (-0.1e1 / 0.2e1) + 0.1e1) * (Lf * mu__dot + phi__dot * Wf / 0.2e1 - z__dot) + Karb_f * phi * Wf;
    Fz__fl = -0.5e0 * K__es_f * (reg_fact * (-h__rc_f - Lf * mu - Lf * (h__rc_r - h__rc_f) / (Lf + Lr) + phi * Wf / 0.2e1 + z - stroke__f / 0.2e1) * (0.1e1 + reg_fact ^ 2 * (-h__rc_f - Lf * mu - Lf * (h__rc_r - h__rc_f) / (Lf + Lr) + phi * Wf / 0.2e1 + z - stroke__f / 0.2e1) ^ 2) ^ (-0.1e1 / 0.2e1) + 0.1e1) * (-h__rc_f - Lf * mu - Lf * (h__rc_r - h__rc_f) / (Lf + Lr) + phi * Wf / 0.2e1 + z - stroke__f / 0.2e1) - 0.5e0 * C__es_f * (reg_fact * (-h__rc_f - Lf * mu - Lf * (h__rc_r - h__rc_f) / (Lf + Lr) + phi * Wf / 0.2e1 + z - stroke__f / 0.2e1) * (0.1e1 + reg_fact ^ 2 * (-h__rc_f - Lf * mu - Lf * (h__rc_r - h__rc_f) / (Lf + Lr) + phi * Wf / 0.2e1 + z - stroke__f / 0.2e1) ^ 2) ^ (-0.1e1 / 0.2e1) + 0.1e1) * (-Lf * mu__dot + phi__dot * Wf / 0.2e1 + z__dot) + m * g * Lr / (0.2e1 * Lf + 0.2e1 * Lr) - m * Omega * u * Lr / (Lf + Lr) * h__rc_f / Wf + Ks__f * (h__rc_f + Lf * mu + Lf * (h__rc_r - h__rc_f) / (Lf + Lr) - phi * Wf / 0.2e1 - z) + 0.5e0 * K__es_f * (reg_fact * (h__rc_f + Lf * mu + Lf * (h__rc_r - h__rc_f) / (Lf + Lr) - phi * Wf / 0.2e1 - z - stroke__f / 0.2e1) * (0.1e1 + reg_fact ^ 2 * (h__rc_f + Lf * mu + Lf * (h__rc_r - h__rc_f) / (Lf + Lr) - phi * Wf / 0.2e1 - z - stroke__f / 0.2e1) ^ 2) ^ (-0.1e1 / 0.2e1) + 0.1e1) * (h__rc_f + Lf * mu + Lf * (h__rc_r - h__rc_f) / (Lf + Lr) - phi * Wf / 0.2e1 - z - stroke__f / 0.2e1) + 0.5e0 * C__es_f * (reg_fact * (h__rc_f + Lf * mu + Lf * (h__rc_r - h__rc_f) / (Lf + Lr) - phi * Wf / 0.2e1 - z - stroke__f / 0.2e1) * (0.1e1 + reg_fact ^ 2 * (h__rc_f + Lf * mu + Lf * (h__rc_r - h__rc_f) / (Lf + Lr) - phi * Wf / 0.2e1 - z - stroke__f / 0.2e1) ^ 2) ^ (-0.1e1 / 0.2e1) + 0.1e1) * (Lf * mu__dot - phi__dot * Wf / 0.2e1 - z__dot) - 0.5e0 * Cs__f__r * (reg_fact * (Lf * mu__dot - phi__dot * Wf / 0.2e1 - z__dot) * (0.1e1 + reg_fact ^ 2 * (Lf * mu__dot - phi__dot * Wf / 0.2e1 - z__dot) ^ 2) ^ (-0.1e1 / 0.2e1) - 0.1e1) * (Lf * mu__dot - phi__dot * Wf / 0.2e1 - z__dot) + 0.5e0 * Cs__f__b * (reg_fact * (Lf * mu__dot - phi__dot * Wf / 0.2e1 - z__dot) * (0.1e1 + reg_fact ^ 2 * (Lf * mu__dot - phi__dot * Wf / 0.2e1 - z__dot) ^ 2) ^ (-0.1e1 / 0.2e1) + 0.1e1) * (Lf * mu__dot - phi__dot * Wf / 0.2e1 - z__dot) - Karb_f * phi * Wf;


    % if Fz__rr>=0
        VerticalLoads.Fz_rr = Fz__rr;
    % else
    %     VerticalLoads.Fz_rr = 0;
    % end

    % if Fz__rl>=0
        VerticalLoads.Fz_rl = Fz__rl;
    % else
    %     VerticalLoads.Fz_rl = 0;
    % end

    % if Fz__fr>=0
        VerticalLoads.Fz_fr = Fz__fr;
    % else
    %     VerticalLoads.Fz_fr = 0;
    % end

    % if Fz__fl>=0
        VerticalLoads.Fz_fl = Fz__fl;
    % else
    %     VerticalLoads.Fz_fl = 0;
    % end

end

