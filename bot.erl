%% An Erlang robot on the core binding supplied by the arena.
%% Precise contacts lead shots; remembered passive cues guide the search.
%% Clunkyard robot API 2.0.0: one core under every language, the same
%% answers everywhere. Facts (a call shows its arguments):
%% match: mode ruleset control robots is_3d tick_rate max_ticks
%%   robot_id(slot)
%% clock: tick seconds remaining_ticks
%% arena: size gravity terrain_kind ground_height(x y)
%% me: health position velocity heading pitch roll angular_velocity
%%   attitude grounded altitude applied
%% battery: charge capacity regen sustainable_drive compute
%%   drive_cost(fraction)
%% drive: hover_lift
%% gun: aim_now cooldown_ticks spread max_power damage_scale
%%   can_fire(power reserve) shot_speed(power) shot_cost(power)
%%   reload_ticks(power) shot_damage(power) power_for_speed(speed)
%%   power_for_flight_time(range seconds) lead(contact power)
%%   lead_point(position velocity power)
%% scanner: last proximity_range reach(arc) arc_for_reach(range)
%% sensors: contacts nearest cues contact(slot) position_of(contact)
%% tracker: cue nearest_bearing_rate velocity(slot) position(slot)
%%   last_seen(slot)
%% nav: zone zone.margin(point) zone.contains(point margin)
%%   zone.distance_from(point) storm storm.margin(point)
%%   storm.contains(point margin) storm.distance_from(point) hill
%%   hill.margin(point) hill.contains(point margin)
%%   hill.distance_from(point) pickups nearest_pickup(kind)
%%   field_distance(a b)
%% score: me best armour
%% angles: normalize(degrees) angle_diff(target current)
%%   bearing_to(from to) elevation_to(from to) direction(heading pitch)
%%   distance(a b) clamp(value low high)
%% raw: number(path) text(path) has(path) flag(path) count(path) line()
%% Controls: drive.set(throttle turn pitch_turn)
%%   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
%%   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
%%   tracker.configure(smoothing forget_ticks cue_ticks)
%%   status.set(text value) raw.extra(json)
%% Every control argument is optional; a second call keeps what the first
%% set. Absent facts are your language's none. Names are snake_case here;
%% your language cases them its own way. Angles are degrees from the +x
%% axis; pitch is positive up.
%% Manual: https://clunkyard.com/manual#sdk
-module(bot).
-export([start/0,on_start/1,on_turn/2]).
-define(U64,16#FFFFFFFFFFFFFFFF).
start()->clunkyard:run(?MODULE),halt().
on_start(R)->clunkyard:match_is_3d(R) andalso maps:get(z,clunkyard:arena_gravity(R))==0.
clamp(Value)->clunkyard:angles_clamp(Value,-1,1).
steer(Towards,Current)->clamp(clunkyard:angles_angle_diff(Towards,Current)/45).
on_turn(R,Free)->
    Position=clunkyard:me_position(R),Heading=clunkyard:me_heading(R),Pitch=clunkyard:me_pitch(R),
    clunkyard:drive_forces(R,#{brake=>0.2}),
    case {clunkyard:sensors_nearest(R),clunkyard:tracker_cue(R)} of
        {undefined,undefined}->
            Scan=clunkyard:scanner_last(R),Tick=clunkyard:clock_tick(R),
            Up=case Free of true->((Tick div 10) rem 3-1)*35;false->0 end,
            clunkyard:drive_set(R,#{throttle=>0.5,turn=>0.33,pitch_turn=>steer(0,Pitch)}),
            clunkyard:scanner_set(R,#{heading=>clunkyard:angles_normalize(maps:get(heading,Scan)+36),arc=>40,pitch=>Up}),
            clunkyard:status_set(R,#{text=><<"SEARCH">>});
        {undefined,Cue}->
            Heard=maps:get(position,Cue),Bearing=clunkyard:angles_bearing_to(Position,Heard),
            Distance=clunkyard:angles_distance(Position,Heard),Tick=clunkyard:clock_tick(R),
            Elevation=case Free of true->clunkyard:angles_elevation_to(Position,Heard);false->0 end,
            Error=min(maps:get(bearing_error,Cue),30),Across=(Tick rem 5-2)/2*Error,
            Up=case Free of true->((Tick div 5) rem 3-1)*Error/2;false->0 end,
            clunkyard:drive_set(R,#{throttle=>0.7,turn=>steer(Bearing,Heading),pitch_turn=>steer(Elevation,Pitch)}),
            clunkyard:scanner_set(R,#{heading=>Bearing+Across,arc=>clunkyard:scanner_arc_for_reach(R,Distance+100),pitch=>Elevation+Up}),
            clunkyard:status_set(R,#{text=><<"CUE">>,value=>Distance});
        {Target,_}->
            Range=maps:get(range,Target),Bearing=maps:get(bearing,Target),Elevation=maps:get(elevation,Target),
            Throttle=if Range>350->0.7;Range<200->-0.4;true->0.2 end,
            clunkyard:drive_set(R,#{throttle=>Throttle,turn=>steer(Bearing,Heading),pitch_turn=>steer(Elevation,Pitch)}),
            clunkyard:scanner_set(R,#{heading=>Bearing,arc=>20,pitch=>Elevation}),
            Power=clunkyard:gun_power_for_flight_time(R,Range,1),
            {AimHeading,AimPitch}=case clunkyard:gun_lead(R,Target,Power) of undefined->{Bearing,Elevation};Aim->{maps:get(heading,Aim),maps:get(pitch,Aim)}end,
            clunkyard:gun_aim(R,#{heading=>AimHeading,pitch=>AimPitch}),Now=clunkyard:gun_aim_now(R),
            case abs(clunkyard:angles_angle_diff(AimHeading,maps:get(heading,Now)))<3 andalso abs(AimPitch-maps:get(pitch,Now))<3 andalso clunkyard:gun_can_fire(R,Power,10) of
                true->clunkyard:gun_fire(R,#{power=>Power});false->ok
            end,
            clunkyard:status_set(R,#{text=><<"TRACK">>,value=>Range})
    end,
    case clunkyard:battery_charge(R)<clunkyard:battery_capacity(R)/4 of
        true->clunkyard:drive_set(R,#{throttle=>clunkyard:drive_get(R,throttle)*0.5});false->ok
    end,
    case clunkyard:nav_storm(R) of
        undefined->ok;
        Storm->case clunkyard:nav_storm_margin(R,Position)>-60 of
            true->Away=clunkyard:angles_bearing_to(maps:get(center,Storm),Position),clunkyard:drive_set(R,#{throttle=>0.8,turn=>clamp(clunkyard:angles_angle_diff(Away,Heading)/30)});
            false->ok end
    end,
    case clunkyard:nav_zone(R) of
        undefined->ok;
        Zone->case clunkyard:nav_zone_margin(R,Position)<60 of
            true->Centre=maps:get(center,Zone),Climb=case maps:get(shape,Zone) of <<"sphere">>->clunkyard:angles_elevation_to(Position,Centre);_->0 end,
                clunkyard:drive_set(R,#{throttle=>0.8,turn=>clamp(clunkyard:angles_angle_diff(clunkyard:angles_bearing_to(Position,Centre),Heading)/30),pitch_turn=>clamp((Climb-Pitch)/30)});
            false->ok end
    end,
    %% The arena's rate calibration, xorshift-sqrt v1; safe to delete.
    case clunkyard:raw_number(R,<<"state.calibration.work">>) of
        Work when is_number(Work),Work>0->clunkyard:status_set(R,#{text=><<"WORK">>,value=>calibration_work(trunc(Work),88172645463325252 bxor clunkyard:clock_tick(R),0.0)});
        _->ok
    end,
    Free.
calibration_work(0,X,Sum)->(floor(Sum)+(X bsr 40)) rem 1000000;
calibration_work(N,X0,Sum)->
    X1=X0 bxor ((X0 bsl 13) band ?U64),X2=X1 bxor (X1 bsr 7),X3=X2 bxor ((X2 bsl 17) band ?U64),
    calibration_work(N-1,X3,Sum+math:sqrt(X3 bsr 11)).
