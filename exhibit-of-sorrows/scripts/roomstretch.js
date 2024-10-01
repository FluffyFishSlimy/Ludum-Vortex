function setupRoomStretch(e,t,o){gameObjects.exhibit.setBackgroundAtIndex(t,"bgs","bg6"),gameObjects.roomStretchObjs||(gameObjects.roomStretchObjs={roomIndex:t,roomContainer:o,lastSoundUpdate:0,shouldUpdate:!1,armMinDist:80,roomUnlocked:!1,doDarkCleanup:!1,roomCompleted:!1,accumulateStatic:0,accumulateFlash:0,dollImages:[],dollBodyList:[],dollPosX:-1,dollPosY:-1,doHorrorSection:!1,streamersList:[],loosenAmt:0}),gameObjects.roomStretchObjs.cleanupButton=new Button(globalScene,gameObjects.roomStretchObjs.roomContainer,()=>{},{atlas:"buttons",ref:"glow",alpha:.001,x:0,y:9999,scaleX:1.25,scaleY:1.25},{atlas:"buttons",ref:"glow",alpha:.001,scaleX:1.5,scaleY:1.5},{atlas:"buttons",ref:"glow",alpha:.001,scaleX:1.5,scaleY:1.5}),gameObjects.roomStretchObjs.cleanupButton.setOnMouseDownFunc(stretchCleanup),gameObjects.roomStretchObjs.frame1=e.add.image(-340,325,"roomStretch","framesStretch1"),gameObjects.roomStretchObjs.frame1x=e.add.image(-340,325,"roomStretch","framesStretch1x"),gameObjects.roomStretchObjs.frame2=e.add.image(-15,180,"roomStretch","framesStretch2"),gameObjects.roomStretchObjs.frame2x=e.add.image(-15,180,"roomStretch","framesStretch3x"),gameObjects.roomStretchObjs.frame3=e.add.image(-65,380,"roomStretch","framesStretch3"),gameObjects.roomStretchObjs.frame1.alpha=0,gameObjects.roomStretchObjs.frame1x.alpha=0,gameObjects.roomStretchObjs.frame2.alpha=0,gameObjects.roomStretchObjs.frame2x.alpha=0,gameObjects.roomStretchObjs.frame3.alpha=0,o.add(gameObjects.roomStretchObjs.frame1),o.add(gameObjects.roomStretchObjs.frame1x),o.add(gameObjects.roomStretchObjs.frame2),o.add(gameObjects.roomStretchObjs.frame2x),o.add(gameObjects.roomStretchObjs.frame3),gameObjects.roomStretchObjs.doll=-1,setStretchDollPos(-236,gameVars.height-161),setStretchDollImage("dollNeutral"),setStretchDollImage("dollAnxious"),setStretchDollImage("dollFearful"),setStretchDollImage("dollNeutral"),setStretchDollImage("dollExpectant"),setStretchDollImage("dollHappy"),gameObjects.roomStretchObjs.touchspot=e.add.image(204,380,"roomStretch","touchspot"),o.add(gameObjects.roomStretchObjs.touchspot);for(let t=0;t<8;t++){let s="streamer"+(t+1),r=e.add.image(0,-90,"roomStretch",s).setOrigin(.5,0);gameObjects.roomStretchObjs.streamersList.push(r),o.add(r)}let s=-525;for(let e=0;e<7;e++)gameObjects.roomStretchObjs.streamersList[e].x=s,s+=.5*gameObjects.roomStretchObjs.streamersList[e].width+.5*gameObjects.roomStretchObjs.streamersList[e+1].width+20;gameObjects.roomStretchObjs.streamersList[7].x=s,gameObjects.roomStretchObjs.armseg1=e.add.image(gameObjects.roomStretchObjs.dollPosX+35.5,gameObjects.roomStretchObjs.dollPosY-85,"roomStretch","armseg"),gameObjects.roomStretchObjs.armseg1.rotation=-.3,gameObjects.roomStretchObjs.armseg1.setOrigin(.02,.5),o.add(gameObjects.roomStretchObjs.armseg1);let r,a,m,c=gameObjects.roomStretchObjs.armseg1.x+94*Math.cos(gameObjects.roomStretchObjs.armseg1.rotation),b=gameObjects.roomStretchObjs.armseg1.y+94*Math.sin(gameObjects.roomStretchObjs.armseg1.rotation);gameObjects.roomStretchObjs.armseg2=e.add.image(c,b,"roomStretch","armseg"),gameObjects.roomStretchObjs.armseg2.setOrigin(.029,.5),o.add(gameObjects.roomStretchObjs.armseg2),gameObjects.roomStretchObjs.elbow=e.add.image(c,b,"roomStretch","hand"),gameObjects.roomStretchObjs.elbow.alpha=0,gameObjects.roomStretchObjs.elbow.velX=0,gameObjects.roomStretchObjs.elbow.velY=0,gameObjects.roomStretchObjs.elbow.scaleX=.01,gameObjects.roomStretchObjs.elbow.scaleY=.01,o.add(gameObjects.roomStretchObjs.elbow),gameObjects.roomStretchObjs.hand=e.add.image(-120,630,"roomStretch","hand"),gameObjects.roomStretchObjs.hand.scaleX=.8,gameObjects.roomStretchObjs.hand.scaleY=.8,gameObjects.roomStretchObjs.hand.velX=0,gameObjects.roomStretchObjs.hand.velY=0,o.add(gameObjects.roomStretchObjs.hand),gameObjects.roomStretchObjs.handButton=new Button(e,{container:o,normal:{atlas:"buttons",ref:"glow",x:0,y:gameVars.halfHeight,alpha:.01,scaleX:1,scaleY:1},hover:{atlas:"buttons",ref:"glow",alpha:.02,scaleX:1.02,scaleY:1.02},disable:{atlas:"buttons",ref:"glow",alpha:.02,scaleX:1e-4,scaleY:1e-4},isDraggable:!0,onDrop:()=>{}}),gameObjects.roomStretchObjs.placard=new Button(e,o,()=>{gameVars.horrorPoint?gameObjects.roomStretchObjs.roomCompleted?updateInfoText("..."):updateInfoText("Ms. Stretch"):gameVars.darkPoint?updateInfoText("What is the furthest she could reach?"):updateInfoText("Ms. Stretch")},{atlas:"buttons",ref:"placard",x:0,y:gameVars.height-73},{atlas:"buttons",ref:"placard_hover"}),setTimeout(()=>{addToUpdateFuncList(roomStretchUpdate)},500),messageBus.subscribe("exhibitMove",e=>{if(e===t){if(tweenVolume("gladiator0",.1),tweenVolume("gladiator1",.8),tweenVolume("gladiator2",.2),gameObjects.roomStretchObjs.shouldUpdate=!0,!gameObjects.roomStretchObjs.oneTimeStreamers){gameObjects.roomStretchObjs.oneTimeStreamers=!0;for(let e=0;e<8;e++)gameObjects.roomStretchObjs.streamersList[e].scaleY=.8+.4*Math.random(),gameObjects.roomStretchObjs.streamersList[e].velY=.1*(Math.random()-.5);addToUpdateFuncList(updateStreamers),setTimeout(()=>{removeFromUpdateFuncList(updateStreamers)},2e4)}}else setTimeout(()=>{gameObjects.roomStretchObjs.shouldUpdate=!1},700)}),r=messageBus.subscribe("startDarkSequence",e=>{r.unsubscribe(),gameObjects.roomStretchObjs.frame1.alpha=1,gameObjects.roomStretchObjs.frame2.alpha=1,gameObjects.roomStretchObjs.frame3.alpha=1,gameObjects.roomStretchObjs.cleanupButton.setPos(gameObjects.roomStretchObjs.touchspot.x,gameObjects.roomStretchObjs.touchspot.y)}),a=messageBus.subscribe("startHorrorSequence",e=>{gameObjects.roomStretchObjs.doDarkCleanup=!1,gameObjects.roomStretchObjs.roomUnlocked=!1,a.unsubscribe(),gameObjects.roomStretchObjs.handButton.setState("normal")}),m=messageBus.subscribe("startTrueStretchHorror",e=>{gameObjects.roomStretchObjs.doDarkCleanup=!1,gameObjects.roomStretchObjs.roomUnlocked=!1,setStretchDollPos(-295,gameVars.height-161),gameObjects.roomStretchObjs.touchspot.x=355,gameObjects.roomStretchObjs.touchspot.y=185,gameObjects.roomStretchObjs.armseg1.x=gameObjects.roomStretchObjs.dollPosX+35.5,m.unsubscribe(),gameObjects.roomStretchObjs.doHorrorSection=!0,gameObjects.roomStretchObjs.frame1.destroy(),gameObjects.roomStretchObjs.frame1x.alpha=1,pullbackStreamers()})}function roomStretchUpdate(e){if(!gameObjects.roomStretchObjs.shouldUpdate)return;let t=gameVars.height-152,o=gameObjects.roomStretchObjs.armMinDist,s=.01+43e-5*o,r=!gameObjects.roomStretchObjs.roomUnlocked||gameObjects.roomStretchObjs.doDarkCleanup||gameObjects.roomStretchObjs.roomCompleted;gameObjects.roomStretchObjs.elbow.velX*=.96,gameObjects.roomStretchObjs.elbow.velY*=.96,gameObjects.roomStretchObjs.elbow.y+=gameObjects.roomStretchObjs.elbow.velY*e,gameObjects.roomStretchObjs.elbow.x+=gameObjects.roomStretchObjs.elbow.velX*e;let a=t-.25*gameObjects.roomStretchObjs.elbow.height*gameObjects.roomStretchObjs.elbow.scaleY;gameObjects.roomStretchObjs.elbow.y>a&&(gameObjects.roomStretchObjs.elbow.y=a,gameObjects.roomStretchObjs.elbow.velY=-.1,gameObjects.roomStretchObjs.elbow.velX>0?gameObjects.roomStretchObjs.elbow.velX=Math.max(0,.98*gameObjects.roomStretchObjs.elbow.velX-.2):gameObjects.roomStretchObjs.elbow.velX<0&&(gameObjects.roomStretchObjs.elbow.velX=Math.min(0,.98*gameObjects.roomStretchObjs.elbow.velX+.2))),r||(gameObjects.roomStretchObjs.hand.x=gameObjects.roomStretchObjs.touchspot.x,gameObjects.roomStretchObjs.hand.velX*=.5,gameObjects.roomStretchObjs.hand.y=gameObjects.roomStretchObjs.touchspot.y,gameObjects.roomStretchObjs.hand.velY*=.5),gameObjects.roomStretchObjs.hand.velX*=.96,gameObjects.roomStretchObjs.hand.velY*=.96,gameObjects.roomStretchObjs.hand.y+=gameObjects.roomStretchObjs.hand.velY*e,gameObjects.roomStretchObjs.hand.x+=gameObjects.roomStretchObjs.hand.velX*e;let m=t-.25*gameObjects.roomStretchObjs.hand.height*gameObjects.roomStretchObjs.hand.scaleY;if(gameObjects.roomStretchObjs.hand.y>m-10&&gameObjects.roomStretchObjs.hand.y>m&&(gameObjects.roomStretchObjs.hand.y=m,gameObjects.roomStretchObjs.hand.velY=-.1,gameObjects.roomStretchObjs.hand.velX>0?gameObjects.roomStretchObjs.hand.velX=Math.max(0,.95*gameObjects.roomStretchObjs.hand.velX-.25):gameObjects.roomStretchObjs.hand.velX<0&&(gameObjects.roomStretchObjs.hand.velX=Math.min(0,.95*gameObjects.roomStretchObjs.hand.velX+.25))),gameObjects.roomStretchObjs.handButton.getIsDragged()?(gameObjects.roomStretchObjs.hand.scaleX=.83,gameObjects.roomStretchObjs.hand.scaleY=.83):(gameObjects.roomStretchObjs.hand.scaleX=.8,gameObjects.roomStretchObjs.hand.scaleY=.8),gameObjects.roomStretchObjs.handButton.getIsDragged()){gameObjects.roomStretchObjs.elbow.velY+=.08;let e=gameObjects.roomStretchObjs.handButton.getXPos(),t=gameObjects.roomStretchObjs.handButton.getYPos(),o=e-gameObjects.roomStretchObjs.hand.x,s=t-gameObjects.roomStretchObjs.hand.y,r=Math.sqrt(o*o+s*s);if(r>70){let e=70/r;o*=e,s*=e}gameObjects.roomStretchObjs.hand.velX+=.034*o-.14*gameObjects.roomStretchObjs.hand.velX,gameObjects.roomStretchObjs.hand.velY+=.034*s-.14*gameObjects.roomStretchObjs.hand.velY}else r&&(gameObjects.roomStretchObjs.elbow.velY+=.22*e,gameObjects.roomStretchObjs.hand.velY+=.28*e,gameObjects.roomStretchObjs.handButton.setPos(gameObjects.roomStretchObjs.hand.x,gameObjects.roomStretchObjs.hand.y));let c=gameObjects.roomStretchObjs.overstretched?26:23,b=gameObjects.roomStretchObjs.hand.x-c*Math.cos(gameObjects.roomStretchObjs.hand.rotation)-gameObjects.roomStretchObjs.elbow.x,O=gameObjects.roomStretchObjs.hand.y-c*Math.sin(gameObjects.roomStretchObjs.hand.rotation)-gameObjects.roomStretchObjs.elbow.y,j=Math.sqrt(b*b+O*O);if(j>o){let t=j-o,r=t/o,a=b*r*s,m=O*r*s;gameObjects.roomStretchObjs.hand.velX-=a,gameObjects.roomStretchObjs.hand.velY-=m,gameObjects.roomStretchObjs.elbow.velX+=a,gameObjects.roomStretchObjs.elbow.velY+=m,(o+=.05*t*e)<110&&(o+=.015*t*e);let c=gameObjects.roomStretchObjs.hand.rotation;Math.abs(gameObjects.roomStretchObjs.hand.rotation-gameObjects.roomStretchObjs.armseg2.rotation)>Math.PI&&(gameObjects.roomStretchObjs.hand.rotation>gameObjects.roomStretchObjs.armseg2.rotation?c=gameObjects.roomStretchObjs.hand.rotation-2*Math.PI:gameObjects.roomStretchObjs.hand.rotation<gameObjects.roomStretchObjs.armseg2.rotation&&(c=gameObjects.roomStretchObjs.hand.rotation+2*Math.PI)),gameObjects.roomStretchObjs.hand.rotation=.9*c+.1*gameObjects.roomStretchObjs.armseg2.rotation}else!gameObjects.roomStretchObjs.overstretched&&o>82&&!gameObjects.roomStretchObjs.handButton.getIsDragged()&&(o=.96*o-2*e);let l=gameObjects.roomStretchObjs.elbow.x-gameObjects.roomStretchObjs.armseg1.x,h=gameObjects.roomStretchObjs.elbow.y-gameObjects.roomStretchObjs.armseg1.y,g=Math.sqrt(l*l+h*h);if(g>o){let e=(g-o)/o,t=l*e*s,r=h*e*s;gameObjects.roomStretchObjs.elbow.velX-=2*t,gameObjects.roomStretchObjs.elbow.velY-=2*r}gameObjects.roomStretchObjs.armseg1.rotation=Math.atan2(h,l),gameObjects.roomStretchObjs.armseg1.scaleX=.01*g;let S=91+.02*o,d=gameObjects.roomStretchObjs.armseg1.x+Math.cos(gameObjects.roomStretchObjs.armseg1.rotation)*S*gameObjects.roomStretchObjs.armseg1.scaleX,n=gameObjects.roomStretchObjs.armseg1.y+Math.sin(gameObjects.roomStretchObjs.armseg1.rotation)*S*gameObjects.roomStretchObjs.armseg1.scaleX;if(gameObjects.roomStretchObjs.armseg2.x=d,gameObjects.roomStretchObjs.armseg2.y=n,gameObjects.roomStretchObjs.armseg2.rotation=Math.atan2(O,b),gameObjects.roomStretchObjs.armseg2.scaleX=.0113*j-.035,r){let t,s=0;o>225&&gameObjects.roomStretchObjs.doHorrorSection?(o>290&&gameObjects.roomStretchObjs.loosenAmt<.15&&(gameObjects.roomStretchObjs.loosenAmt+=33e-6*e),s=26e-6*o+.83-gameObjects.roomStretchObjs.loosenAmt-.007*(e-1),o>339.5&&(console.log("She can't stretch any further"),t*=.999)):gameObjects.roomStretchObjs.doHorrorSection?s=.0035*o+.088:(s=.0048*o+.032,o>218&&(s*=2)),t=o-s*e;let r=gameObjects.roomStretchObjs.armMinDist;if(gameObjects.roomStretchObjs.armMinDist=Math.max(gameObjects.roomStretchObjs.overstretched?230:80,t),o>120){gameObjects.roomStretchObjs.armMinDist>r+.02&&updateStretchSounds(o);let e=35e-5*(o-120);o>210&&(e+=.0045*(o-210)),gameObjects.roomStretchObjs.armseg1.scaleY=1-e,gameObjects.roomStretchObjs.armseg2.scaleY=1-e}}if(gameObjects.roomStretchObjs.doHorrorSection&&o>220&&gameObjects.roomStretchObjs.handButton.getIsDragged()&&!gameObjects.roomStretchObjs.roomCompleted)if(Math.random()<4e-4*(o-130)+gameObjects.roomStretchObjs.accumulateStatic-.0012)if(gameObjects.roomStretchObjs.accumulateStatic=0,o>260&&Math.random()<gameObjects.roomStretchObjs.accumulateFlash-.4)gameObjects.roomStretchObjs.accumulateFlash=0,showFlashRand(2),gameObjects.roomStretchObjs.armMinDist+=5;else if(Math.random()<.4){showStaticRand(1,void 0,void 0,.0025*(o-220)),gameObjects.roomStretchObjs.armMinDist+=1.9,gameObjects.roomStretchObjs.accumulateFlash*=.5}else gameObjects.roomStretchObjs.armMinDist+=.5,gameObjects.roomStretchObjs.accumulateFlash+=.1*e;else gameObjects.roomStretchObjs.accumulateStatic;let i=gameObjects.roomStretchObjs.touchspot.x-gameObjects.roomStretchObjs.hand.x,u=gameObjects.roomStretchObjs.touchspot.y-gameObjects.roomStretchObjs.hand.y,p=i*i+u*u;if(p<10500&&!gameObjects.roomStretchObjs.doDarkCleanup)if(Math.abs(i)+Math.abs(u)<20){if(!gameObjects.roomStretchObjs.roomUnlocked){if(gameObjects.roomStretchObjs.roomUnlocked=!0,gameObjects.roomStretchObjs.doHorrorSection){gameObjects.roomStretchObjs.roomCompleted=!0,setTimeout(()=>{removeFromUpdateFuncList(roomStretchUpdate)},1e4),gameObjects.roomStretchObjs.frame2.destroy(),gameObjects.roomStretchObjs.frame2x.alpha=1,gameObjects.roomStretchObjs.handButton.destroy(),playSound("tear6"),showStaticLite(9,10,2),showAltReality(["stretch2","stretch3","stretch4","stretch4","stretch5","stretch5","stretch6"],1.2),gameObjects.sounds.pumpamb.stop(),gameObjects.roomStretchObjs.hand.alpha=0,gameObjects.roomStretchObjs.hand.x=gameObjects.roomStretchObjs.dollPosX+25,gameObjects.roomStretchObjs.hand.y=500;let e=globalScene.add.image(gameObjects.roomStretchObjs.touchspot.x,gameObjects.roomStretchObjs.touchspot.y,"roomStretch","hand");gameObjects.roomStretchObjs.roomContainer.add(e),gameObjects.roomStretchObjs.armseg1.x=e.x-25,gameObjects.roomStretchObjs.armseg1.y=e.y+7,gameObjects.roomStretchObjs.handButton.setState("disable")}else{gameVars.horrorPoint||gameObjects.roomStretchObjs.handButton.setState("disable"),setStretchDollImage("dollHappy",!0);let e=globalScene.add.image(gameObjects.roomStretchObjs.dollPosX-65,gameObjects.roomStretchObjs.dollPosY-125,"roomStretch","exclamation");gameObjects.roomStretchObjs.roomContainer.add(e),e.scaleX=.5,e.scaleY=.5,globalScene.tweens.timeline({targets:e,tweens:[{scaleX:1.2,scaleY:1.2,duration:150,ease:"Quad.easeOut"},{scaleX:0,scaleY:0,duration:450,ease:"Quad.easeIn"}]}),globalScene.tweens.timeline({targets:e,tweens:[{x:gameObjects.roomStretchObjs.dollPosX-100,y:gameObjects.roomStretchObjs.dollPosY-140,ease:"Quad.easeOut"}]})}setTimeout(()=>{if(gameVars.horrorPoint&&!gameObjects.roomStretchObjs.doHorrorSection){let e=globalScene.add.image(gameObjects.roomStretchObjs.dollPosX-100,gameObjects.roomStretchObjs.dollPosY-30,"buttons","key_yellow");playSound("keyfound"),gameObjects.roomStretchObjs.roomContainer.add(e),setTimeout(()=>{showStaticRand(1),setTimeout(()=>{e.destroy(),gameObjects.roomStretchObjs.doDarkCleanup=!0,showStaticRand(3,void 0,()=>{showFlashRand(3),messageBus.publish("startTrueStretchHorror")})},550)},800)}else createKey(gameObjects.roomStretchObjs.dollPosX-100,gameObjects.roomStretchObjs.dollPosY-30,gameObjects.roomStretchObjs.roomIndex,gameObjects.roomStretchObjs.roomContainer,!gameVars.horrorPoint),gameVars.horrorPoint&&setTimeout(()=>{gameObjects.generalDarkness.alpha=.1,setTimeout(()=>{gameObjects.generalDarkness.alpha=.04,setTimeout(()=>{gameObjects.generalDarkness.alpha=.15,setTimeout(()=>{gameObjects.generalDarkness.alpha=.07},100)},350)},100)},350)},gameVars.horrorPoint?700:300)}}else{let e=p<2200?45:25;gameObjects.roomStretchObjs.doHorrorSection&&(e*=4),gameObjects.roomStretchObjs.hand.velX+=i*e/p,gameObjects.roomStretchObjs.hand.velY+=u*e/p}if(gameObjects.roomStretchObjs.roomCompleted)setStretchDollImage("dollDefeated");else if(gameObjects.roomStretchObjs.roomUnlocked);else if(gameObjects.roomStretchObjs.overstretched)if(gameObjects.sounds.pumpamb.volume=Math.max(.01,(o-260)/90),o<285){setStretchDollImage("dollAnxious");let e=1.4*Math.random();gameObjects.roomStretchObjs.doll.x=gameObjects.roomStretchObjs.dollPosX-.7+e+9}else if(o<340){playSoundOnce("tear4",250),setStretchDollImage("dollFearful"),gameObjects.roomStretchObjs.doll.rotation+=.035*(Math.random()-.25);let e=1.7*Math.random()-.85;gameObjects.roomStretchObjs.doll.x=gameObjects.roomStretchObjs.dollPosX+e+9,gameObjects.roomStretchObjs.dollBody.x=gameObjects.roomStretchObjs.dollPosX+e,gameObjects.roomStretchObjs.armseg1.x=gameObjects.roomStretchObjs.dollPosX+35.5+e}else{playSoundOnce("tear5",250),setStretchDollImage("dollHorrified"),gameObjects.roomStretchObjs.doll.rotation+=.16*(Math.random()-.25);let e=3*Math.random()-1.5;gameObjects.roomStretchObjs.doll.x=gameObjects.roomStretchObjs.dollPosX+e+9,gameObjects.roomStretchObjs.dollBody.x=gameObjects.roomStretchObjs.dollPosX+e,gameObjects.roomStretchObjs.armseg1.x=gameObjects.roomStretchObjs.dollPosX+35.5+e}else o<120?setStretchDollImage("dollNeutral"):o<230?setStretchDollImage("dollExpectant"):o<290?o>260&&!gameObjects.roomStretchObjs.flashFear?(setStretchDollImage("dollFearful"),playSoundOnce("tear2"),showStaticLite(1,4,1.5),setTimeout(()=>{gameObjects.roomStretchObjs.flashFear||(gameObjects.roomStretchObjs.armMinDist+=5),gameObjects.roomStretchObjs.flashFear=!0},150)):(gameObjects.roomStretchObjs.flashStaticOnce||(gameObjects.roomStretchObjs.flashStaticOnce=!0,showStaticLite(2,8,1,.1),gameObjects.roomStretchObjs.armMinDist+=2,setTimeout(()=>{gameObjects.roomStretchObjs.armMinDist+=1,setTimeout(()=>{gameObjects.roomStretchObjs.armMinDist+=1},20)},20)),playSoundOnce("tear1"),setStretchDollImage("dollAnxious")):(gameObjects.roomStretchObjs.overstretched||(gameObjects.roomStretchObjs.overstretched=!0,gameObjects.sounds.pumpamb.play({loop:!0}),gameObjects.sounds.pumpamb.volume=.01,gameObjects.roomStretchObjs.armseg2.setOrigin(.022,.5),gameObjects.roomStretchObjs.armMinDist+=1,setTimeout(()=>{playSoundOnce("tear3"),showAltReality(["stretch1","stretch2","stretch3","stretch4"],1.06),gameObjects.roomStretchObjs.armMinDist+=8},100)),setStretchDollImage("dollFearful"));gameObjects.roomStretchObjs.doll.rotation*=.75}function stretchCleanup(){gameObjects.roomStretchObjs.doDarkCleanup=!0,gameObjects.roomStretchObjs.cleanupButton.destroy(),gameObjects.exhibit.needCleanup=!1,setTimeout(()=>{playSound("deepbell2"),updateInfoTextSoft("Room cleaned up.",2250)},400)}function setStretchDollImage(e,t=!1){if(gameObjects.roomStretchObjs.doll==gameObjects.roomStretchObjs.dollImages[e])return;if(gameObjects.roomStretchObjs.doll.visible=!1,"dollDefeated"===e)return updateDollBodyImage("dollDefeated"),void(gameObjects.roomStretchObjs.doll.visible=!1);updateDollBodyImage("dollBody");if(!gameObjects.roomStretchObjs.dollImages[e]){let t=globalScene.add.image(gameObjects.roomStretchObjs.dollPosX+9,gameObjects.roomStretchObjs.dollPosY+-91,"roomStretch",e);gameObjects.roomStretchObjs.dollImages[e]=t,gameObjects.roomStretchObjs.roomContainer.add(t)}gameObjects.roomStretchObjs.dollImages[e].visible=!0,gameObjects.roomStretchObjs.doll=gameObjects.roomStretchObjs.dollImages[e];gameObjects.roomStretchObjs.doHorrorSection;gameObjects.roomStretchObjs.doll.x=gameObjects.roomStretchObjs.dollPosX+9,gameObjects.roomStretchObjs.doll.y=gameObjects.roomStretchObjs.dollPosY+-91,t?(gameObjects.roomStretchObjs.doll.rotation=.075,gameObjects.roomStretchObjs.dollBody.scaleX=1.01,gameObjects.roomStretchObjs.dollBody.scaleY=1.01,gameObjects.roomStretchObjs.doll.scaleX=1.015,gameObjects.roomStretchObjs.doll.scaleY=1.02,setTimeout(()=>{gameObjects.roomStretchObjs.dollBody.scaleX=1.004,gameObjects.roomStretchObjs.dollBody.scaleY=1.004,gameObjects.roomStretchObjs.doll.scaleX=1.006,gameObjects.roomStretchObjs.doll.scaleY=1.008,setTimeout(()=>{gameObjects.roomStretchObjs.dollBody.scaleX=1.001,gameObjects.roomStretchObjs.dollBody.scaleY=1.001,gameObjects.roomStretchObjs.doll.scaleX=1.002,gameObjects.roomStretchObjs.doll.scaleY=1.003,setTimeout(()=>{gameObjects.roomStretchObjs.dollBody.scaleX=1,gameObjects.roomStretchObjs.dollBody.scaleY=1,gameObjects.roomStretchObjs.doll.scaleX=1,gameObjects.roomStretchObjs.doll.scaleY=1},40)},40)},50)):(gameObjects.roomStretchObjs.doll.rotation=.04,gameObjects.roomStretchObjs.dollBody.scaleY=1.008,gameObjects.roomStretchObjs.doll.scaleY=1.012,setTimeout(()=>{gameObjects.roomStretchObjs.dollBody.scaleY=1.003,gameObjects.roomStretchObjs.doll.scaleY=1.004,setTimeout(()=>{gameObjects.roomStretchObjs.dollBody.scaleY=1,gameObjects.roomStretchObjs.doll.scaleY=1},40)},50))}function setStretchDollPos(e,t){if(gameObjects.roomStretchObjs.dollPosX=e,gameObjects.roomStretchObjs.dollPosY=t,gameObjects.roomStretchObjs.dollBody&&(gameObjects.roomStretchObjs.dollBody.x=gameObjects.roomStretchObjs.dollPosX,gameObjects.roomStretchObjs.dollBody.y=gameObjects.roomStretchObjs.dollPosY),gameObjects.roomStretchObjs.doll){let e=9,t=-91;gameObjects.roomStretchObjs.doll.x=gameObjects.roomStretchObjs.dollPosX+e,gameObjects.roomStretchObjs.doll.y=gameObjects.roomStretchObjs.dollPosY+t}}function updateDollBodyImage(e){gameObjects.roomStretchObjs.dollBodyName!==e&&(gameObjects.roomStretchObjs.dollBodyName=e,gameObjects.roomStretchObjs.dollBody&&(gameObjects.roomStretchObjs.dollBody.visible=!1),gameObjects.roomStretchObjs.dollBodyList[e]?(gameObjects.roomStretchObjs.dollBodyList[e].visible=!0,gameObjects.roomStretchObjs.dollBody=gameObjects.roomStretchObjs.dollBodyList[e]):(gameObjects.roomStretchObjs.dollBody=globalScene.add.image(gameObjects.roomStretchObjs.dollPosX,gameObjects.roomStretchObjs.dollPosY,"roomStretch",e),gameObjects.roomStretchObjs.roomContainer.add(gameObjects.roomStretchObjs.dollBody),gameObjects.roomStretchObjs.dollBodyList[e]=gameObjects.roomStretchObjs.dollBody))}function updateStretchSounds(e){e>120&&(gameObjects.roomStretchObjs.lastSoundUpdate<=0?e<250?(playSound("rubber",5,.35),gameObjects.roomStretchObjs.lastSoundUpdate=80+Math.floor(50*Math.random())):(playSound("rubber",8,1),gameObjects.roomStretchObjs.lastSoundUpdate=60+Math.floor(10*Math.random())):gameObjects.roomStretchObjs.lastSoundUpdate--)}function updateStreamers(){for(let e=0;e<8;e++){let t=gameObjects.roomStretchObjs.streamersList[e];t.velY+=.1*(1-t.scaleY)-.03*t.velY,gameObjects.roomStretchObjs.streamersList[e].scaleY+=.014*t.velY}}function pullbackStreamers(){for(let e=0;e<8;e++){let t=gameObjects.roomStretchObjs.streamersList[e];t.y-=50,5!==e&&6!==e||(t.y-=100),t.scaleY+=.03+.08*Math.random()}addToUpdateFuncList(updateStreamers),setTimeout(()=>{removeFromUpdateFuncList(updateStreamers)},2e4)}