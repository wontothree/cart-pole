import { type Entity, type Handle, InferenceComponent, MotorComponent, Util } from "prototwin";

export class CartPole extends InferenceComponent {
    public cartMotor: Handle<MotorComponent>;
    public poleMotor: Handle<MotorComponent>;

    private direction: number = 1; // 1: 오른쪽, -1: 왼쪽

    constructor(entity: Entity) {
        super(entity);
        this.cartMotor = this.handle(MotorComponent);
        this.poleMotor = this.handle(MotorComponent);
    }

    public override async updateAsync() {
        const cartMotor = this.cartMotor.value;
        const poleMotor = this.poleMotor.value;
        const observations = this.observations;
        if (cartMotor === null || poleMotor === null || observations === null) { return; }

        const cartPosition = cartMotor.currentPosition;
        const cartVelocity = cartMotor.currentVelocity;
        const poleAngularDistance = Util.signedAngularDifference(poleMotor.currentPosition, Math.PI);
        const poleAngularVelocity = poleMotor.currentVelocity;

        // 카트 위치가 기준 위치를 초과하면 방향을 전환합니다.
        if (Math.abs(cartPosition) >= 0.3) {
            this.direction *= -1; // 방향 반전
        }

        // 방향에 따라 속도를 설정합니다.
        cartMotor.targetVelocity = this.direction * 0.5;
    }
}
