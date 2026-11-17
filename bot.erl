%% An Erlang robot on the core binding supplied by the arena.
%% Contacts narrow the beam and lead shots from a line fitted through recent
%% sightings; remembered passive cues guide the search.
%% Bearing is earned: a scan contact's bearing is off by up to its
%% bearing_error (degrees, a fixed share of the arc), so a wide beam finds a
%% robot and a narrow one aims at it. Contacts within 120 units are exact.
%% Clunkyard robot API 2.0.0: one core under every language, the same
%% answers everywhere. Facts (a call shows its arguments):
%% match: mode ruleset control robots is_3d tick_rate max_ticks
%%   robot_id(slot)
%% clock: tick seconds remaining_ticks
%% arena: size gravity terrain_kind ground_height(x y)
%% me: health position velocity heading pitch roll angular_velocity
%%   attitude grounded altitude wall_contact applied hull_mounts
%%   to_mount(heading pitch) from_mount(heading pitch) mount_toward(point)
%% battery: charge capacity regen sustainable_drive compute
%%   drive_cost(fraction)
%% drive: hover_lift
%% gun: aim_now cooldown_ticks spread max_power damage_scale
%%   can_fire(power reserve) shot_speed(power) shot_cost(power)
%%   reload_ticks(power) shot_damage(power) power_for_speed(speed)
%%   power_for_flight_time(range seconds) lead(contact power)
%%   lead_point(position velocity power)
%% scanner: last proximity_range reach(arc) arc_for_reach(range) is_on
%% sensors: contacts nearest cues contact(slot) position_of(contact)
%% tracker: cue nearest_bearing_rate velocity(slot) position(slot)
%%   history(slot) last_seen(slot)
%% nav: zone zone.margin(point) zone.contains(point margin)
%%   zone.distance_from(point) storm storm.margin(point)
%%   storm.contains(point margin) storm.distance_from(point) hill
%%   hill.margin(point) hill.contains(point margin)
%%   hill.distance_from(point) wall_margin() pickups nearest_pickup(kind)
%%   field_distance(a b)
%% score: me best armour
%% angles: normalize(degrees) angle_diff(target current)
%%   bearing_to(from to) elevation_to(from to) direction(heading pitch)
%%   distance(a b) to_chassis(heading pitch attitude)
%%   from_chassis(heading pitch attitude) clamp(value low high)
%% raw: number(path) text(path) has(path) flag(path) count(path) line()
%% Controls: drive.set(throttle turn pitch_turn)
%%   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
%%   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
%%   scanner.power(on) tracker.configure(smoothing forget_ticks cue_ticks)
%%   status.set(text value) debug.log(line) raw.extra(json)
%% Every control argument is optional; a second call keeps what the first
%% set. Absent facts are your language's none. Names are snake_case here;
%% your language cases them its own way. Angles are degrees from the +x
%% axis; pitch is positive up.
%% Manual: https://clunkyard.com/manual#sdk
-module(bot).
-export([start/0,on_start/1,on_turn/2]).
-define(SEARCH_ARC,40).  %% degrees: sweeps the circle in half a second
-define(HOLD_WIDTH,60).  %% units across at the target: it stays in the beam from tick to tick
-define(MIN_ARC,6).      %% degrees: the narrowest the beam holds
-define(WINDOW,12).      %% ticks of sightings the aim averages
-define(SIGHTINGS,8).    %% narrow sightings in the window before a shot
-define(LOOK_EVERY,40).  %% ticks between looks around the target, for a nearer robot
-define(REACQUIRE,10).   %% ticks to look again where a lost target was
start()->clunkyard:run(?MODULE),halt().
on_start(R)->
    Free=clunkyard:match_is_3d(R) andalso maps:get(z,clunkyard:arena_gravity(R))==0,
    #{free=>Free,dodges=>0,dodge_end=>0,slot=>0}.
clamp(Value)->clunkyard:angles_clamp(Value,-1,1).
steer(Towards,Current)->clamp(clunkyard:angles_angle_diff(Towards,Current)/45).
%% A straight line through the narrowest of the last WINDOW ticks' sightings:
%% where the robot is now, how fast it moves, and how many sightings it rests
%% on. Each sighting is off by its bearing_error; the line averages that out.
fit(R,Slot,Tick)->
    Recent=[S||S<-clunkyard:tracker_history(R,Slot),maps:get(tick,S)>Tick-?WINDOW],
    fit(R,Tick,Recent,length(Recent)).
fit(_,_,_,N) when N<2->none;
fit(R,Tick,Recent,_)->
    Best=lists:min([maps:get(bearing_error,S)||S<-Recent]),
    Seen=[S||S<-Recent,maps:get(bearing_error,S)=<1.5*Best],N=length(Seen),
    case N<2 of
        true->none;
        false->
            T0=lists:sum([maps:get(tick,S)||S<-Seen])/N,
            Spread=lists:sum([math:pow(maps:get(tick,S)-T0,2)||S<-Seen]),
            Line=fun(Axis)->
                Mean=lists:sum([maps:get(Axis,maps:get(position,S))||S<-Seen])/N,
                Slope=lists:sum([(maps:get(tick,S)-T0)*(maps:get(Axis,maps:get(position,S))-Mean)||S<-Seen])/Spread,
                {Mean+Slope*(Tick-T0),Slope*clunkyard:match_tick_rate(R)}
            end,
            [{X,Vx},{Y,Vy},{Z,Vz}]=[Line(Axis)||Axis<-[x,y,z]],
            {#{x=>X,y=>Y,z=>Z},#{x=>Vx,y=>Vy,z=>Vz},N}
    end.
on_turn(R,#{free:=Free,slot:=Slot}=Memory)->
    Position=clunkyard:me_position(R),Heading=clunkyard:me_heading(R),Pitch=clunkyard:me_pitch(R),Tick=clunkyard:clock_tick(R),
    clunkyard:drive_forces(R,#{brake=>0.2}),
    Nearest=clunkyard:sensors_nearest(R),
    LastSeen=case Slot of 0->undefined;_->clunkyard:tracker_last_seen(R,Slot) end,
    Lost=LastSeen=/=undefined andalso Tick-LastSeen=<?REACQUIRE,
    Remembered=case Nearest of undefined->Slot;_->maps:get(slot,Nearest) end,
    case {Nearest,Lost,clunkyard:tracker_cue(R)} of
        {undefined,true,_}->
            %% Lost it: look again where it was last seen, widening the beam, before searching.
            Where=clunkyard:tracker_position(R,Slot),Bearing=clunkyard:angles_bearing_to(Position,Where),
            Elevation=case Free of true->clunkyard:angles_elevation_to(Position,Where);false->0 end,
            clunkyard:drive_set(R,#{throttle=>0.5,turn=>steer(Bearing,Heading),pitch_turn=>steer(Elevation,Pitch)}),
            clunkyard:scanner_set(R,#{heading=>Bearing,arc=>min(?SEARCH_ARC,maps:get(arc,clunkyard:scanner_last(R))*2),pitch=>Elevation}),
            clunkyard:status_set(R,#{text=><<"REACQUIRE">>});
        {undefined,false,undefined}->
            Scan=clunkyard:scanner_last(R),
            Up=case Free of true->((Tick div 10) rem 3-1)*35;false->0 end,
            clunkyard:drive_set(R,#{throttle=>0.5,turn=>0.33,pitch_turn=>steer(0,Pitch)}),
            clunkyard:scanner_set(R,#{heading=>clunkyard:angles_normalize(maps:get(heading,Scan)+36),arc=>?SEARCH_ARC,pitch=>Up}),
            clunkyard:status_set(R,#{text=><<"SEARCH">>});
        {undefined,false,Cue}->
            Heard=maps:get(position,Cue),Bearing=clunkyard:angles_bearing_to(Position,Heard),
            Distance=clunkyard:angles_distance(Position,Heard),
            Elevation=case Free of true->clunkyard:angles_elevation_to(Position,Heard);false->0 end,
            Error=min(maps:get(bearing_error,Cue),30),Across=(Tick rem 5-2)/2*Error,
            Up=case Free of true->((Tick div 5) rem 3-1)*Error/2;false->0 end,
            clunkyard:drive_set(R,#{throttle=>0.7,turn=>steer(Bearing,Heading),pitch_turn=>steer(Elevation,Pitch)}),
            clunkyard:scanner_set(R,#{heading=>Bearing+Across,arc=>clunkyard:scanner_arc_for_reach(R,Distance+100),pitch=>Elevation+Up}),
            clunkyard:status_set(R,#{text=><<"CUE">>,value=>Distance});
        {Target,_,_}->
            Range=maps:get(range,Target),Bearing=maps:get(bearing,Target),Elevation=maps:get(elevation,Target),
            Throttle=if Range>350->0.7;Range<200->-0.4;true->0.2 end,
            clunkyard:drive_set(R,#{throttle=>Throttle,turn=>steer(Bearing,Heading),pitch_turn=>steer(Elevation,Pitch)}),
            %% Halve the beam each tick until it is HOLD_WIDTH across at the target:
            %% narrow enough for a precise bearing, wide enough to keep it in.
            Hold=max(?MIN_ARC,2*math:atan2(?HOLD_WIDTH/2,Range)*180/math:pi()),
            %% In a crowd a narrow beam sees one robot. Every LOOK_EVERY ticks,
            %% widen it for a tick: a nearer robot it finds becomes the target.
            Arc=case Tick rem ?LOOK_EVERY of
                0->?SEARCH_ARC;
                _->max(Hold,min(?SEARCH_ARC,maps:get(arc,clunkyard:scanner_last(R))/2))
            end,
            clunkyard:scanner_set(R,#{heading=>Bearing,arc=>Arc,pitch=>Elevation}),
            Power=clunkyard:gun_power_for_flight_time(R,Range,1),
            %% An exact contact: gun_lead is the intercept if the target holds course.
            %% A noisy one: lead the fitted line, and fire only once it rests on
            %% enough sightings.
            {Aim,Ready}=case maps:get(bearing_error,Target)>0 of
                true->case fit(R,maps:get(slot,Target),Tick) of
                    none->{undefined,false};
                    {FitPosition,FitVelocity,Count}->{clunkyard:gun_lead_point(R,FitPosition,FitVelocity,Power),Count>=?SIGHTINGS}
                end;
                false->{clunkyard:gun_lead(R,Target,Power),true}
            end,
            {AimHeading,AimPitch}=case Aim of undefined->{Bearing,Elevation};_->{maps:get(heading,Aim),maps:get(pitch,Aim)}end,
            clunkyard:gun_aim(R,#{heading=>AimHeading,pitch=>AimPitch}),Now=clunkyard:gun_aim_now(R),
            case Ready andalso abs(clunkyard:angles_angle_diff(AimHeading,maps:get(heading,Now)))<3 andalso abs(AimPitch-maps:get(pitch,Now))<3 andalso clunkyard:gun_can_fire(R,Power,10) of
                true->clunkyard:gun_fire(R,#{power=>Power});false->ok
            end,
            clunkyard:status_set(R,#{text=>case Ready of true-><<"TRACK">>;false-><<"NARROW">> end,value=>Range})
    end,
    %% A simple dodge: vary it, or predict the shooter.
    Flash=lists:any(fun(Contact)->maps:get(source,Contact)==<<"muzzle">> end,clunkyard:sensors_cues(R)),
    Next=case Tick>=maps:get(dodge_end,Memory) andalso Flash of
        true->Memory#{dodges:=maps:get(dodges,Memory)+1,dodge_end:=Tick+10,slot:=Remembered};
        false->Memory#{slot:=Remembered}
    end,
    case Tick<maps:get(dodge_end,Next) of
        true->
            %% Off the line of fire: left, then right next time (turn is positive left, strafe right).
            Left=maps:get(dodges,Next) rem 2==1,
            case clunkyard:match_control(R) of
                <<"forces">>->clunkyard:drive_set(R,#{throttle=>0.5}),clunkyard:drive_forces(R,#{strafe=>case Left of true->-1;false->1 end});
                _->clunkyard:drive_set(R,#{throttle=>1,turn=>case Left of true->1;false->-1 end})
            end,
            clunkyard:status_set(R,#{text=>case Left of true-><<"dodge:left">>;false-><<"dodge:right">> end,value=>maps:get(dodges,Next)});
        false->ok
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
    Next.
