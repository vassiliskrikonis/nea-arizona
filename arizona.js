var scene = new THREE.Scene();
var bgScene = new THREE.Scene();
var shadowScene = new THREE.Scene();
var modelScene = new THREE.Scene();

var raycaster = new THREE.Raycaster();
var mouse = new THREE.Vector2();

var SCALE = 0.05;

var orthoCamera = new THREE.OrthographicCamera(-window.innerWidth/2, window.innerWidth/2, window.innerHeight/2, -window.innerHeight/2, 1, 3000);
var bgOrthoCamera = new THREE.OrthographicCamera(-window.innerWidth/2, window.innerWidth/2, window.innerHeight/2, -window.innerHeight/2, 1, 3000);
orthoCamera.position.z = 1000;
bgOrthoCamera.position.z = 1000;

var renderer = new THREE.WebGLRenderer({alpha: true});
renderer.setSize( window.innerWidth, window.innerHeight );
renderer.autoClear = false;
renderer.setClearColor( 0xff0000, 0 );
document.body.appendChild( renderer.domElement );

var bgTexture = THREE.ImageUtils.loadTexture('743.jpg');
bgTexture.minFilter = THREE.NearestFilter;
var bgMaterial = new THREE.MeshBasicMaterial({
  map: bgTexture
});
var bgGeometry = new THREE.PlaneGeometry(window.innerWidth, window.innerHeight);
var bg = new THREE.Mesh(bgGeometry, bgMaterial);
bg.position.set(0,0,-1000);
bg.rotation.set(-0.45, 0.53, 0.24);
bgScene.add(bg);

var physicsWorld = new CANNON.World();
physicsWorld.gravity.set(0, -9.82*10, 0); // m/s²
physicsWorld.broadphase = new CANNON.NaiveBroadphase();
physicsWorld.solver.iterations = 10;
var timestep = 1/60;

var solidMaterial = new CANNON.Material('solidMaterial');
var solidOnSolid = new CANNON.ContactMaterial(solidMaterial, solidMaterial, {
  friction: 5
});
physicsWorld.addContactMaterial(solidOnSolid);

function Box(width, height, depth) {
  var geometry = new THREE.BoxGeometry(width, height, depth);
  var material = new THREE.MeshBasicMaterial({
    color: 0xffffff,
    wireframe: false
  }); 
  this.body = new CANNON.Body({
    mass: 50,
    material: solidMaterial,
    shape: new CANNON.Box(new CANNON.Vec3(width/2, height/2, depth/2)) // CANNON uses haflextends, vectors from the center mass of the shape thus resulting in double the vector values
  });
  // physicsWorld.addBody(this.body);
  THREE.Mesh.call(this, geometry, material);
}
Box.prototype = Object.create(THREE.Mesh.prototype);
Box.prototype.constructor = Box;

var GroundColumn = augment.extend(Box, {
  constructor: function() {
    Box.call(this, 200*SCALE, 2800*SCALE, 200*SCALE);
    // this.body = new CANNON.Body({
    //   mass: 1000,
    //   shape: new CANNON.Box(new CANNON.Vec3(200*SCALE/2,2800*SCALE/2,200*SCALE/2)), // CANNON uses haflextends, vectors from the center mass of the shape thus resulting in double the vector values
    //   // position: new CANNON.Vec3(0,0,0)
    //   // quaternion: new CANNON.Quaternion(-4, 3, 4)
    // });
    // physicsWorld.addBody(this.body);
    return this;
  }
});
GroundColumn.positions = [
  100, 1400, 3900,
  2900, 1400, 3900,
  5700, 1400, 3900,
  100, 1400, 2000,
  2900, 1400, 2000, 
  5700, 1400, 2000,
  100, 1400, 100,
  2900, 1400, 100,
  5700, 1400, 100
];

var FloorPart = augment.extend(Box, {
  constructor: function() {
    Box.call(this, 2900*SCALE, 200*SCALE, 2000*SCALE);
    return this;
  }
});
FloorPart.positions = [
  1450, 2900, 1000, // 10nth part
  4350, 2900, 1000,
  1450, 2900, 3000,
  4350, 2900, 3000,
  1450, 5300, 1000,
  4350, 5300, 1000,
  1450, 5300, 3000,
  4350, 5300, 3000
];

var FloorColumn = augment.extend(Box, {
  constructor: function() {
    Box.call(this, 200*SCALE, 2200*SCALE, 200*SCALE);
    return this;
  }
});
FloorColumn.positions = [
  100, 4100, 3900, //18th part
  2900, 4100, 3900,
  5700, 4100, 3900,
  100, 4100, 2000,
  2900, 4100, 2000, 
  5700, 4100, 2000,
  100, 4100, 100,
  2900, 4100, 100,
  5700, 4100, 100,
  //roof
  2900, 6500, 100,
  5700, 6500, 100,
  2900, 6500, 2000,
  5700, 6500, 2000
];

var Roof = augment.extend(Box, {
  constructor: function() {
    Box.call(this, 3000*SCALE, 200*SCALE, 2100*SCALE);
    return this;
  }
});
Roof.positions = [
  4300, 7700, 1050 //31st part
];

var Balcony = augment.extend(Box, {
  constructor: function() {
    Box.call(this, 1000*SCALE, 200*SCALE, 600*SCALE);
    this.body.mass = 20;
    return this;
  }
});
Balcony.positions = [
  1450, 2900, -300, //32nd part
  4350, 2900, -300,
  1450, 2900, 4300,
  4350, 2900, 4300,
  1450, 5300, -300, 
  4350, 5300, -300,
  1450, 5300, 4300,
  4350, 5300, 4300
];

var components = [
  'GroundColumn',
  'FloorPart',
  'FloorColumn',
  'Roof',
  'Balcony'
];

var constraints = [
  // BodyB -> BodyA (in children indeces -1)
  31, 9,
  32, 10,
  33, 11,
  34, 12,
  35, 13,
  36, 14,
  37, 15,
  38, 16
];
var connectedBodies = [];


for(var componentIndex in components) {
  var component = window[components[componentIndex]];

  for(var i=0, l = component.positions.length; i<l; i=i+3) {
    var part = new component();
    part.position.set(component.positions[i]*SCALE, component.positions[i+1]*SCALE, component.positions[i+2]*SCALE);
    part.body.position.copy(part.position);
    physicsWorld.addBody(part.body);
    part.edges = new THREE.EdgesHelper(part, 0xff0000);
    scene.add(part);
    var shadowPart = new component();
    shadowPart.position.copy(part.position);
    shadowScene.add(shadowPart);
  }
}
for(var i=0, l = scene.children.length; i<l; i++) {
  if(scene.children[i].edges)
    scene.add(scene.children[i].edges);
}

for (var i=0,l = constraints.length; i<l; i=i+2) {
  var bodyB = scene.children[constraints[i]].body; //balcony
  var bodyA = scene.children[constraints[i+1]].body; //floor
  var pivotB = bodyB.pointToLocalFrame(bodyA.position);
  var pivotA = new CANNON.Vec3(0,0,0);

  var constraint = new CANNON.LockConstraint(bodyB, bodyA);
  // var constraint = new CANNON.DistanceConstraint(bodyB, bodyA);
  // var constraint = new CANNON.PointToPointConstraint(bodyB, pivotB, bodyA, pivotA);

  console.log(bodyB.position + ' to ' + bodyA.pointToLocalFrame(bodyB.position))
  physicsWorld.addConstraint(constraint);

  // bodyA.addShape(bodyB.shapes[0], bodyA.pointToLocalFrame(bodyB.position));
  // physicsWorld.removeBody(bodyB);

  // connectedBodies.push(scene.children[constraints[i]]);
  // connectedBodies.push(scene.children[constraints[i+1]]);
  // physicsWorld.removeBody(bodyB);
}


var groundBody = new CANNON.Body({
  shape: new CANNON.Plane(),
  mass: 0,
});
groundBody.position.y = 0;
groundBody.quaternion.setFromAxisAngle(new CANNON.Vec3(1,0,0),-Math.PI/2);
physicsWorld.addBody(groundBody);


var camera = orthoCamera;
camera.position.set(1022, 760, 1541);
camera.rotation.set(-0.45, 0.53, 0.24);
bgOrthoCamera.position.set(1022, 760, 1541);
bgOrthoCamera.rotation.set(-0.45, 0.53, 0.24);
camera.zoom = 0.41;
var cameraControls = new THREE.OrbitControls(camera);

// var axisHelper = new THREE.AxisHelper(2000);
// scene.add(axisHelper);

var render = function () {
  requestAnimationFrame( render );
  // cameraControls.update();

  // Help by http://stackoverflow.com/a/27832177/3004364
  var gl = renderer.context;
  gl.enable(gl.STENCIL_TEST);
  gl.stencilOp(gl.KEEP, gl.KEEP, gl.REPLACE);
  gl.stencilFunc(gl.ALWAYS, 1, 0xFF);

  renderer.render(shadowScene, camera);

  // Clear the color and depth but NOT the stencil
  renderer.clear(true, true, false);

  // Set the stencil up so we'll only draw if there's a 1 in the stencil buffer.
  gl.stencilOp(gl.KEEP, gl.KEEP, gl.KEEP);//  sfail, spassdfail, spassdepthpass);
  gl.stencilFunc(gl.EQUAL, 1, 0xFF); //func, ref, mask) 

  renderer.render(bgScene, bgOrthoCamera);
  gl.disable(gl.STENCIL_TEST); 

  renderer.render(scene, camera);

  updatePhysics();

};

renderer.domElement.addEventListener('click', function(event) {
  // calculate mouse position in normalized device coordinates
  // (-1 to +1) for both components

  mouse.x = ( event.clientX / window.innerWidth ) * 2 - 1;
  mouse.y = - ( event.clientY / window.innerHeight ) * 2 + 1;

  // update the picking ray with the camera and mouse position  
  raycaster.setFromCamera( mouse, camera ); 

  // calculate objects intersecting the picking ray
  var intersects = raycaster.intersectObjects( scene.children );

  for ( var i = 0; i < intersects.length; i++ ) {

    // intersects[ i ].object.material.color.set( 0xff0000 );  
    var intersected =  intersects[i].object;

    checkConnected(intersected.body);

    physicsWorld.removeBody(intersected.body);
    if(intersected.edges)
      scene.remove(intersected.edges);
    scene.remove(intersected);   
  }
  // END of raycaster code
}, false);

render();

function checkConnected(body) {
  for ( var i in physicsWorld.constraints) {
    var constraint = physicsWorld.constraints[i];
    if( constraint.bodyA === body || constraint.bodyB === body)
      physicsWorld.removeConstraint(constraint);
  }
}

function updatePhysics() {
  physicsWorld.step(timestep);
  for (var i in scene.children) {
    var child = scene.children[i];
    if(child.body != undefined && child.hasOwnProperty('body')) {
      child.position.copy(child.body.position);
      child.quaternion.copy(child.body.quaternion);
    }
  }
}